package dev.example.multiagent;

import dev.langchain4j.agentic.Agent;
import dev.langchain4j.agentic.AgenticServices;
import dev.langchain4j.model.chat.ChatModel;
import dev.langchain4j.service.SystemMessage;
import dev.langchain4j.service.UserMessage;
import dev.langchain4j.service.V;

import java.util.Map;
import java.util.Set;

/**
 * 链式 handoff 的客服台。
 *
 * <pre>
 *   用户
 *    │
 *    ▼
 *  triage ──billing──► human
 *    │                  ▲
 *    └──tech────────────┘
 * </pre>
 *
 * 实线是允许列表里的边。triage 不能直接跳到 human，billing 和 tech 也不能互相移交。
 * 达到 {@code done} 或跳数上限后，最后一份 {@link Handoff#answer()} 成为 {@link SupportDesk#handle} 的返回值。
 *
 * <p>脚本实现不调用模型，用来看控制权怎么走。{@link #withModel(ChatModel)} 换成真正的 LLM 专员，编排器不变。
 */
public final class ChainedHandoffExample {

    /**
     * 移交图就是授权边界。模型只能在自己那一行里点名，不能凭提示词调用图外的代理。
     * {@code done} 不用写进来，规划器会单独识别。
     */
    public static final Map<String, Set<String>> SUPPORT_GRAPH = Map.of(
            "triage", Set.of("billing", "tech"),
            "billing", Set.of("human"),
            "tech", Set.of("human"),
            "human", Set.of());

    private ChainedHandoffExample() {
    }

    public static SupportDesk scripted() {
        return desk("triage", 4, SUPPORT_GRAPH,
                new ScriptedTriage(), new ScriptedBilling(), new ScriptedTech(), new ScriptedHuman());
    }

    /** 四个专员共用一个模型。生产里可以把人工之前的专员换成更便宜的模型，只要返回值仍是 {@link Handoff}。 */
    public static SupportDesk withModel(ChatModel model) {
        return desk("triage", 4, SUPPORT_GRAPH,
                AgenticServices.agentBuilder(TriageAgent.class).chatModel(model).build(),
                AgenticServices.agentBuilder(BillingAgent.class).chatModel(model).build(),
                AgenticServices.agentBuilder(TechAgent.class).chatModel(model).build(),
                AgenticServices.agentBuilder(HumanAgent.class).chatModel(model).build());
    }

    /**
     * {@code planner} 传入的是 Supplier：每次根调用都要一个新的规划器，跳数才不会串到下一通对话。
     * 根代理的 {@code outputKey} 是 {@code answer}。规划器 {@code done(文本)} 时，框架把这段文本写进该键并返回。
     */
    public static SupportDesk desk(String entryAgentName,
                                   int maxInvocations,
                                   Map<String, Set<String>> allowedTargets,
                                   Object... agents) {
        return AgenticServices.plannerBuilder(SupportDesk.class)
                .name("chainedHandoff")
                .subAgents(agents)
                .outputKey("answer")
                .planner(() -> new HandoffPlanner(entryAgentName, maxInvocations, allowedTargets))
                .build();
    }

    public interface TriageAgent {

        @SystemMessage("""
                你是客服分诊，不直接办理业务。
                只返回 JSON，字段为 answer 和 nextAgent。
                nextAgent 只能是 billing、tech 或 done。
                billing：发票、退款、账单、扣费。
                tech：登录、密码、报错、无法使用。
                done：寒暄或一句话能回答的问题，此时 answer 就是给用户的话。
                移交时 answer 写给下一个专员，一两句说明即可。
                """)
        @UserMessage("用户说：{{request}}")
        @Agent(name = "triage",
                description = "判断交给 billing、tech，或直接 done",
                outputKey = "handoff")
        Handoff triage(@V("request") String request);
    }

    public interface BillingAgent {

        @SystemMessage("""
                你是账单专员，只处理发票、退款、账单和扣费。
                只返回 JSON，字段为 answer 和 nextAgent。
                nextAgent 只能是 human 或 done。
                能处理就 done，answer 是给用户的答复。
                用户明确要求人工，或属于投诉升级，才交给 human，answer 写清卡在哪里。
                """)
        @UserMessage("""
                用户说：{{request}}
                分诊移交说明：{{priorAnswer}}
                """)
        @Agent(name = "billing",
                description = "处理发票、退款和账单；处理不了就移交 human",
                outputKey = "handoff")
        Handoff billing(@V("request") String request, @V("priorAnswer") String priorAnswer);
    }

    public interface TechAgent {

        @SystemMessage("""
                你是技术专员，只处理登录、密码、报错和功能无法使用。
                只返回 JSON，字段为 answer 和 nextAgent。
                nextAgent 只能是 human 或 done。
                能处理就 done，answer 是给用户的操作步骤。
                用户明确要求人工，或缺少账号信息无法继续，才交给 human。
                """)
        @UserMessage("""
                用户说：{{request}}
                分诊移交说明：{{priorAnswer}}
                """)
        @Agent(name = "tech",
                description = "处理登录和故障；处理不了就移交 human",
                outputKey = "handoff")
        Handoff tech(@V("request") String request, @V("priorAnswer") String priorAnswer);
    }

    public interface HumanAgent {

        @SystemMessage("""
                你是人工坐席，负责给出最终答复。
                只返回 JSON，字段为 answer 和 nextAgent。
                nextAgent 固定写 done。
                answer 面向用户，吸收上游说明，不要再建议转交。
                """)
        @UserMessage("""
                用户说：{{request}}
                由 {{handedOffFrom}} 移交。
                上游说明：{{priorAnswer}}
                """)
        @Agent(name = "human",
                description = "人工坐席，接手后直接答复",
                outputKey = "handoff")
        Handoff human(@V("request") String request,
                      @V("priorAnswer") String priorAnswer,
                      @V("handedOffFrom") String handedOffFrom);
    }

    /** 下面四个类是确定性替身，方便在没有密钥时观察移交顺序。 */
    public static class ScriptedTriage {
        @Agent(name = "triage",
                description = "账单交给 billing，登录和报错交给 tech，其余直接答复",
                outputKey = "handoff")
        public Handoff triage(@V("request") String request) {
            String text = request == null ? "" : request;
            if (text.contains("发票") || text.contains("账单") || text.contains("退款")) {
                return Handoff.to("billing", "分诊：账单问题。");
            }
            if (text.contains("登录") || text.contains("密码") || text.contains("报错")) {
                return Handoff.to("tech", "分诊：技术问题。");
            }
            return Handoff.done("分诊：可以直接答复。你好，我是客服入口。");
        }
    }

    public static class ScriptedBilling {
        @Agent(name = "billing",
                description = "处理发票和退款；用户要求人工时移交 human",
                outputKey = "handoff")
        public Handoff billing(@V("request") String request, @V("priorAnswer") String priorAnswer) {
            if (request != null && request.contains("人工")) {
                return Handoff.to("human", "账单专员处理不了。上下文：" + priorAnswer);
            }
            return Handoff.done("账单专员：发票将在 1 个工作日内发送到预留邮箱。");
        }
    }

    public static class ScriptedTech {
        @Agent(name = "tech",
                description = "处理登录和故障；用户要求人工时移交 human",
                outputKey = "handoff")
        public Handoff tech(@V("request") String request, @V("priorAnswer") String priorAnswer) {
            if (request != null && request.contains("人工")) {
                return Handoff.to("human", "技术专员：请先重置密码。这个问题需要人工。");
            }
            return Handoff.done("技术专员：请重置密码后重新登录。");
        }
    }

    public static class ScriptedHuman {
        @Agent(name = "human",
                description = "人工坐席，接手后直接答复",
                outputKey = "handoff")
        public Handoff human(@V("request") String request, @V("priorAnswer") String priorAnswer) {
            return Handoff.done("人工坐席已接手。上游说明：" + priorAnswer);
        }
    }
}
