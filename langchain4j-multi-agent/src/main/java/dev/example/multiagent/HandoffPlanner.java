package dev.example.multiagent;

import dev.langchain4j.agentic.planner.AgentInstance;
import dev.langchain4j.agentic.planner.InitPlanningContext;
import dev.langchain4j.agentic.planner.Planner;
import dev.langchain4j.agentic.planner.Action;
import dev.langchain4j.agentic.planner.PlanningContext;
import dev.langchain4j.agentic.scope.AgentInvocation;
import dev.langchain4j.agentic.scope.AgenticScope;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * 链式 handoff。当前专员返回后，本规划器只根据它声明的 {@code nextAgent} 决定下一家，
 * 然后调用那一家；上一个专员不会被再问一次。
 *
 * <p>这和 {@code SupervisorAgent} 的差别在控制权。主管每次都回到规划模型，由它重新挑选 subagent。
 * 这里的规划器是确定性的：模型只负责在自己的返回值里指出下一个名字，允许列表和跳数上限由代码守住。
 *
 * <p>状态在每次 subagent 返回后写入 scope。进程在两次调用之间重启时，{@link #firstAction} 会从
 * 尚未执行的那一家接着跑。正在执行的那一家如果中途崩溃，恢复时会再跑一遍，所以有副作用的工具要自己做到幂等。
 */
public final class HandoffPlanner implements Planner {

    private final String entryAgentName;
    private final int maxInvocations;
    private final Map<String, Set<String>> allowedTargets;

    private Map<String, AgentInstance> agentsByName = Map.of();
    private int invocations;
    /** 恢复执行时要调用的专员。冷启动为 null。 */
    private String pendingAgent;

    /**
     * @param entryAgentName  第一家专员的 {@code @Agent(name)}
     * @param maxInvocations  一条请求里最多调用几次专员，用来截断互相移交
     * @param allowedTargets  key 是当前专员，value 是它有权移交到的名字；{@code done} 不必写进集合
     */
    public HandoffPlanner(String entryAgentName, int maxInvocations, Map<String, Set<String>> allowedTargets) {
        if (entryAgentName == null || entryAgentName.isBlank()) {
            throw new IllegalArgumentException("entryAgentName is required");
        }
        if (maxInvocations < 1) {
            throw new IllegalArgumentException("maxInvocations must be >= 1");
        }
        this.entryAgentName = entryAgentName;
        this.maxInvocations = maxInvocations;
        this.allowedTargets = copy(allowedTargets);
    }

    @Override
    public void init(InitPlanningContext initPlanningContext) {
        // 框架对每一次根调用都 new 一个 Planner，然后再 init。这里仍把字段复位，避免复用实例时串状态。
        Map<String, AgentInstance> indexed = new LinkedHashMap<>();
        for (AgentInstance agent : initPlanningContext.subagents()) {
            AgentInstance previous = indexed.put(agent.name(), agent);
            if (previous != null) {
                throw new IllegalStateException("Duplicate agent name: " + agent.name());
            }
        }
        this.agentsByName = Map.copyOf(indexed);
        this.invocations = 0;
        this.pendingAgent = null;
    }

    @Override
    public Map<String, Object> executionState() {
        Map<String, Object> state = new LinkedHashMap<>();
        state.put("invocations", invocations);
        if (pendingAgent != null) {
            state.put("pendingAgent", pendingAgent);
        }
        return state;
    }

    @Override
    public void restoreExecutionState(Map<String, Object> state) {
        // init 已经把字段清零；恢复必须发生在 init 之后，框架确实是这个顺序。
        Object savedInvocations = state.get("invocations");
        if (savedInvocations instanceof Number number) {
            invocations = number.intValue();
        }
        Object savedPending = state.get("pendingAgent");
        if (savedPending instanceof String name && !name.isBlank()) {
            pendingAgent = name;
        }
    }

    @Override
    public Action firstAction(PlanningContext planningContext) {
        if (pendingAgent != null) {
            // 上次已经决定了下一家，但那一家还没跑完。不要回到入口，也不要再加一次跳数。
            return call(require(pendingAgent));
        }
        invocations = 1;
        pendingAgent = entryAgentName;
        return call(require(entryAgentName));
    }

    @Override
    public Action nextAction(PlanningContext planningContext) {
        AgentInvocation previous = planningContext.previousAgentInvocation();
        if (previous == null || previous.skipped()) {
            pendingAgent = null;
            return done();
        }
        if (!(previous.output() instanceof Handoff handoff)) {
            // 这家专员没有参加 handoff 协议，它的原始返回值就是最终结果。
            pendingAgent = null;
            return done(previous.output());
        }
        if (handoff.finished() || invocations >= maxInvocations) {
            pendingAgent = null;
            return done(visibleAnswer(handoff));
        }

        String requested = handoff.nextAgent().trim();
        AgentInstance target = find(requested);
        // 名字必须同时通过两道门：图里真有这家，而且当前专员被允许移交过去。
        // 模型编造的名字在这里被吃掉，用户仍能拿到目前为止的说明。
        if (target == null
                || requested.equalsIgnoreCase(previous.agentName())
                || !isAllowed(previous.agentName(), target.name())) {
            pendingAgent = null;
            return done(visibleAnswer(handoff));
        }

        invocations++;
        pendingAgent = target.name();
        passContext(planningContext.agenticScope(), previous.agentName(), handoff);
        return call(target);
    }

    /**
     * 下一家通过 {@code @V} 读取这两份状态。写入放在调用之前，所以上一家不需要知道下一家的方法签名。
     * 空白字符串会被 scope 当成“没有这个参数”，所以缺省说明要用一句看得见的占位。
     */
    private static void passContext(AgenticScope scope, String fromAgent, Handoff handoff) {
        scope.writeState("priorAnswer", visibleAnswer(handoff));
        scope.writeState("handedOffFrom", fromAgent);
    }

    private static String visibleAnswer(Handoff handoff) {
        String answer = handoff.answer();
        if (answer == null || answer.isBlank()) {
            return "（上游没有留下说明）";
        }
        return answer;
    }

    private boolean isAllowed(String from, String to) {
        Set<String> targets = allowedTargets.get(from);
        if (targets == null) {
            return false;
        }
        for (String target : targets) {
            if (target.equalsIgnoreCase(to)) {
                return true;
            }
        }
        return false;
    }

    private AgentInstance require(String name) {
        AgentInstance found = find(name);
        if (found == null) {
            throw new IllegalStateException("No subagent named '" + name + "'. Known: " + agentsByName.keySet());
        }
        return found;
    }

    private AgentInstance find(String name) {
        if (name == null) {
            return null;
        }
        AgentInstance exact = agentsByName.get(name);
        if (exact != null) {
            return exact;
        }
        for (Map.Entry<String, AgentInstance> entry : agentsByName.entrySet()) {
            if (entry.getKey().equalsIgnoreCase(name)) {
                return entry.getValue();
            }
        }
        return null;
    }

    private static Map<String, Set<String>> copy(Map<String, Set<String>> allowedTargets) {
        Map<String, Set<String>> copied = new LinkedHashMap<>();
        allowedTargets.forEach((from, targets) -> copied.put(from, Set.copyOf(targets)));
        return Map.copyOf(copied);
    }
}
