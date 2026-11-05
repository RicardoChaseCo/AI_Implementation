import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import java.util.concurrent.ConcurrentHashMap

/**
 * 三十分钟白板版。
 *
 * 条件边是 [route]。环是 [agentLoop] 的 while。
 * 工具节点是 for (call in turn.calls)。检查点是 [Memory]。
 * 人工打断是 State.waiting。并行子代理是 [fanOut] 里的 async。
 * 模型网关的档位是 [model] 的 task 参数。桩模型只负责把这条路径走通。
 *
 * 真接模型时把 [model] 换成 HTTP。循环不用改。
 * 依赖只有 kotlinx-coroutines-core。不引用 LangGraph。
 */

data class Msg(val role: String, val content: String)
data class Call(val id: String, val name: String, val args: Map<String, String>)
data class Task(val role: String, val brief: String)

data class State(
    val threadId: String,
    val messages: List<Msg> = emptyList(),
    val plan: List<Task> = emptyList(),
    val planned: Boolean = false,
    val results: List<String> = emptyList(),
    val waiting: Call? = null,
    val waitingRole: String? = null,
    val doneCalls: Set<String> = emptySet(),
)

class Tool(
    val name: String,
    val write: Boolean,
    val run: (Map<String, String>) -> String,
)

val tools: Map<String, Tool> = mapOf(
    "search" to Tool("search", write = false) { "命中：登录页在 LoginPage.kt" },
    "edit_file" to Tool("edit_file", write = true) { "已写入 ${it["path"]}" },
)

class Memory {
    private val rows = ConcurrentHashMap<String, State>()
    fun save(s: State) { rows[s.threadId] = s }
    fun load(id: String): State = rows[id] ?: State(id)
}

data class Turn(
    val text: String? = null,
    val calls: List<Call> = emptyList(),
    val plan: List<Task> = emptyList(),
)

fun model(task: String, role: String, messages: List<Msg>): Turn {
    val last = messages.lastOrNull()?.content
    val sawTool = messages.any { it.role == "tool" }
    if (task == "planner") {
        return Turn(plan = listOf(Task("frontend", "改登录页"), Task("backend", "改登录接口")))
    }
    return when (role) {
        "frontend" ->
            if (!sawTool) Turn(calls = listOf(Call("f1", "search", mapOf("q" to "登录页"))))
            else Turn(text = "登录页按检索结果改文案")
        "backend" -> when {
            last == "用户拒绝" -> Turn(text = "未修改接口")
            !sawTool -> Turn(calls = listOf(Call("b1", "edit_file", mapOf("path" to "LoginApi.kt"))))
            else -> Turn(text = "登录接口已修改")
        }
        else -> Turn(text = "")
    }
}

fun route(s: State): String = when {
    s.waiting != null -> "halt"
    !s.planned -> "planner"
    s.results.size < s.plan.size -> "subagents"
    else -> "join"
}

fun exec(call: Call, done: Set<String>): Pair<String, Set<String>> {
    if (call.id in done) return "已执行 ${call.id}" to done
    return tools.getValue(call.name).run(call.args) to (done + call.id)
}

suspend fun agentLoop(s0: State, role: String, mem: Memory, out: Channel<String>): State {
    var s = s0
    while (true) {
        mem.save(s)
        if (s.waiting != null) return s
        val turn = model("agent", role, s.messages)
        s = s.copy(messages = s.messages + Msg("assistant", turn.text.orEmpty()))
        if (turn.calls.isEmpty()) {
            out.send("$role: ${turn.text.orEmpty()}")
            mem.save(s)
            return s
        }
        for (call in turn.calls) {
            if (tools.getValue(call.name).write) {
                out.send("等待批准 ${call.name}")
                return s.copy(waiting = call).also { mem.save(it) }
            }
            val (text, done) = exec(call, s.doneCalls)
            out.send("$role · ${call.name}: $text")
            s = s.copy(doneCalls = done, messages = s.messages + Msg("tool", text))
        }
    }
}

fun planNode(s: State): State =
    s.copy(planned = true, plan = model("planner", "planner", s.messages).plan)

suspend fun fanOut(s: State, mem: Memory, out: Channel<String>): State = coroutineScope {
    val pending = s.plan.filter { task -> s.results.none { it.startsWith(task.role) } }
    val pieces = pending.map { task ->
        async {
            val child = agentLoop(
                State("${s.threadId}:${task.role}", messages = listOf(Msg("user", task.brief))),
                task.role,
                mem,
                out,
            )
            task.role to child
        }
    }.awaitAll()
    val paused = pieces.firstOrNull { it.second.waiting != null }
    val finished = pieces.filter { it.second.waiting == null }
    s.copy(
        results = s.results + finished.map { (role, child) ->
            "$role: ${child.messages.last().content}"
        },
        waiting = paused?.second?.waiting,
        waitingRole = paused?.first,
    )
}

suspend fun invoke(threadId: String, userText: String, mem: Memory, out: Channel<String>): State {
    val prev = mem.load(threadId)
    if (prev.waiting != null) return prev
    var s = prev.copy(
        messages = prev.messages + Msg("user", userText),
        planned = false,
        plan = emptyList(),
        results = emptyList(),
        waiting = null,
        waitingRole = null,
    )
    while (true) {
        mem.save(s)
        s = when (route(s)) {
            "halt" -> return s
            "planner" -> planNode(s)
            "subagents" -> fanOut(s, mem, out)
            else -> return s.also { out.send("最终 " + it.results.joinToString(" | ")) }
        }
    }
}

suspend fun resume(threadId: String, approved: Boolean, mem: Memory, out: Channel<String>): State {
    val parent = mem.load(threadId)
    val call = parent.waiting ?: return parent
    val role = parent.waitingRole ?: return parent
    val child0 = mem.load("$threadId:$role")
    val (text, done) = if (approved) exec(call, child0.doneCalls) else "用户拒绝" to child0.doneCalls
    val child = agentLoop(
        child0.copy(waiting = null, doneCalls = done, messages = child0.messages + Msg("tool", text)),
        role,
        mem,
        out,
    )
    val merged = parent.copy(
        waiting = null,
        waitingRole = null,
        results = parent.results + "$role: ${child.messages.last().content}",
    )
    mem.save(merged)
    out.send("最终 " + merged.results.joinToString(" | "))
    return merged
}

fun main() = runBlocking {
    val mem = Memory()
    val out = Channel<String>(Channel.UNLIMITED)
    val printer = launch { for (line in out) println(line) }
    val paused = invoke("t1", "登录页和接口一起改", mem, out)
    println("停住 ${paused.waiting?.name} / ${paused.waitingRole}")
    resume("t1", approved = true, mem, out)
    resume("t1", approved = true, mem, out)
    out.close()
    printer.join()
}
