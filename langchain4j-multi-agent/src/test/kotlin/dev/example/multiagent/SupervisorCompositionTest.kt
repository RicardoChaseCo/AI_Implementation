package dev.example.multiagent

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class SupervisorCompositionTest {

    @Test
    fun `一次 supervisor invoke 里面交替调用规划模型和子代理`() {
        val trace = mutableListOf<String>()
        val supervisor = SupervisorComposition(
            planner = SupervisorComposition.Planner { _, request, lastResponse ->
                when {
                    lastResponse.isEmpty() ->
                        SupervisorComposition.Decision("orderLookup", mapOf("request" to request))
                    lastResponse.startsWith("订单") ->
                        SupervisorComposition.Decision("policy", mapOf("request" to request))
                    else ->
                        SupervisorComposition.Decision("done", response = "可以退。$lastResponse")
                }
            },
            subagents = mapOf(
                "orderLookup" to SupervisorComposition.SubAgent {
                    // 普通方法，没有自己的模型调用。
                    "订单 10086：已发货"
                },
                "policy" to SupervisorComposition.SubAgent {
                    // 这个子代理自己的模型调用，以及它自己的 tool call，都结束在 return 之前。
                    trace += "policy-model"
                    trace += "policy-tool:refundRule"
                    trace += "policy-model"
                    "签收 7 天内未拆封可退"
                },
            ),
        )

        val answer = supervisor.invoke("订单 10086 能退吗", trace)

        assertEquals("可以退。签收 7 天内未拆封可退", answer)
        assertEquals(
            listOf(
                "planner-model",
                "subagent:orderLookup",
                "planner-model",
                "subagent:policy",
                "policy-model",
                "policy-tool:refundRule",
                "policy-model",
                "planner-model",
            ),
            trace,
        )
    }

    @Test
    fun `handoff 的外层函数只调用子代理，中间没有规划模型`() {
        val trace = mutableListOf<String>()
        val desk = HandoffComposition(
            entry = "triage",
            allowed = mapOf("triage" to setOf("billing")),
            agents = mapOf(
                "triage" to HandoffComposition.HandoffAgent { _, _ ->
                    trace += "triage-model"
                    HandoffStep.to("billing", "分诊：账单问题。")
                },
                "billing" to HandoffComposition.HandoffAgent { _, prior ->
                    trace += "billing-model"
                    HandoffStep.done("账单专员已处理。上游：$prior")
                },
            ),
        )

        val answer = desk.invoke("我要开发票", trace)

        assertEquals("账单专员已处理。上游：分诊：账单问题。", answer)
        assertEquals(
            listOf(
                "subagent:triage",
                "triage-model",
                "subagent:billing",
                "billing-model",
            ),
            trace,
        )
    }
}
