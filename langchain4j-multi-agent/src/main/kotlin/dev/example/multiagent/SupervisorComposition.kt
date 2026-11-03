package dev.example.multiagent

/**
 * 把 LangChain4j 藏在 `supervisor.invoke()` 里面的循环摊开。
 *
 * 用户代码里只有一次调用：
 *
 *     supervisor.invoke("订单 10086 能退吗")
 *
 * 这次调用对应 [invoke]。它自己不访问模型。循环在它里面，每一轮有两次彼此分开的调用：
 *
 *  1. [Planner.plan]：规划模型。返回值是名字和参数，不是 tool call。
 *     框架里这是 `PlannerAgent.plan`，由 `SupervisorPlanner.nextSubagent` 发起。
 *  2. [SubAgent.invoke]：Java 按名字调用那一个子代理。
 *     框架里这是 `AgentExecutor.execute`。
 *     子代理是 LLM 时，这次 invoke 内部才有它自己的 `ChatModel.chat`。
 *     子代理挂了 `@Tool` 时，tool call 也只发生在这次 invoke 内部，规划模型看不到。
 *
 * 子代理返回后，循环回到第 1 步。规划模型看到的是上一轮的文本，不是 tool result。
 * 它返回 `done` 时，[invoke] 才把最终字符串交还给用户。
 */
class SupervisorComposition(
    private val planner: Planner,
    private val subagents: Map<String, SubAgent>,
    private val maxRounds: Int = 6,
) {
    fun interface Planner {
        /** 一次规划模型调用。对应 `AiServices` 代理出来的 `PlannerAgent.plan`。 */
        fun plan(agents: String, request: String, lastResponse: String): Decision
    }

    fun interface SubAgent {
        /**
         * 一次子代理调用。非 LLM 代理这里零次模型请求；
         * LLM 代理在函数体里调用自己的模型；有工具时，工具循环也写在函数体里。
         */
        fun invoke(arguments: Map<String, String>): String
    }

    data class Decision(
        val agentName: String,
        val arguments: Map<String, String> = emptyMap(),
        val response: String? = null,
    )

    /**
     * 对应 `PlannerBasedInvocationHandler.PlannerLoop.loop`。
     * 调用方阻塞在这一个函数上，直到规划模型说出 done。
     */
    fun invoke(request: String, trace: MutableList<String> = mutableListOf()): String {
        val scope = linkedMapOf("request" to request)
        var lastResponse = ""
        var round = 0
        while (round < maxRounds) {
            round++
            trace += "planner-model"
            val decision = planner.plan(agentCard(), request, lastResponse)
            if (decision.agentName.equals("done", ignoreCase = true)) {
                return decision.response ?: lastResponse
            }
            val subagent = subagents[decision.agentName]
                ?: throw IllegalStateException("没有名为 ${decision.agentName} 的子代理")
            // 规划模型给出的参数先写入共享 scope，再调用子代理。子代理不接收 Planner 这个对象。
            scope.putAll(decision.arguments)
            trace += "subagent:${decision.agentName}"
            lastResponse = subagent.invoke(scope.toMap())
            scope["lastResponse"] = lastResponse
        }
        return lastResponse
    }

    private fun agentCard(): String =
        subagents.keys.joinToString(", ")
}

/**
 * 子代理交还的结构化结果。字段和 Java 侧的 `Handoff` 相同：
 * `nextAgent` 为 `done` 时，`answer` 是给用户的话。
 */
data class HandoffStep(val answer: String, val nextAgent: String) {
    companion object {
        const val DONE = "done"

        fun done(answer: String) = HandoffStep(answer, DONE)

        fun to(nextAgent: String, noteForNextAgent: String) = HandoffStep(noteForNextAgent, nextAgent)
    }
}

/**
 * 和主管的差别：链路里没有规划模型。
 * 当前子代理的返回值自己带下一个名字，外层函数直接调用那一个。
 * 对应 Java 侧的 `HandoffPlanner.nextAction`。
 */
class HandoffComposition(
    private val entry: String,
    private val agents: Map<String, HandoffAgent>,
    private val allowed: Map<String, Set<String>>,
    private val maxInvocations: Int = 4,
) {
    fun interface HandoffAgent {
        fun invoke(request: String, priorAnswer: String): HandoffStep
    }

    fun invoke(request: String, trace: MutableList<String> = mutableListOf()): String {
        var current = entry
        var prior = ""
        var invocations = 0
        while (invocations < maxInvocations) {
            invocations++
            val agent = agents.getValue(current)
            trace += "subagent:$current"
            val handoff = agent.invoke(request, prior)
            if (handoff.finished()) {
                return handoff.answer
            }
            val next = handoff.nextAgent.trim()
            if (next.equals(current, ignoreCase = true) || next !in allowed.getOrDefault(current, emptySet())) {
                return handoff.answer
            }
            prior = handoff.answer
            current = next
        }
        return prior
    }

    private operator fun Set<String>.contains(name: String): Boolean =
        any { it.equals(name, ignoreCase = true) }

    private fun HandoffStep.finished(): Boolean =
        nextAgent.isBlank() || nextAgent.equals(HandoffStep.DONE, ignoreCase = true)
}
