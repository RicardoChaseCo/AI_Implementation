package dev.example.multiagent;

import dev.langchain4j.agentic.Agent;
import dev.langchain4j.agentic.AgenticServices;
import dev.langchain4j.agentic.supervisor.SupervisorAgent;
import dev.langchain4j.agentic.supervisor.SupervisorContextStrategy;
import dev.langchain4j.agentic.supervisor.SupervisorResponseStrategy;
import dev.langchain4j.model.chat.ChatModel;
import dev.langchain4j.service.SystemMessage;
import dev.langchain4j.service.UserMessage;
import dev.langchain4j.service.V;

/**
 * 主管调度多个 subagent。主管模型每轮看到各 subagent 的名字和描述，选出下一次调用，
 * 调用结束控制权回到主管，直到它给出 {@code done} 或达到 {@code maxAgentsInvocations}。
 *
 * <pre>
 *            ┌─ orderLookup（无模型，查一笔订单事实）
 *   主管 ────┼─ policy（只解释退货政策）
 *            └─ reply（optional：orderFacts 和 policyNote 都齐了才写最终回复）
 * </pre>
 *
 * {@code reply} 标成 {@code optional = true}：主管若在材料齐之前点它，框架会跳过这次调用，
 * 并把“缺了哪个参数”交回主管，而不是让整次请求失败。描述要写清前置条件，主管是靠描述做选择的。
 *
 * <p>{@link SupervisorResponseStrategy#SUMMARY} 让主管根据轨迹写最终答复。
 * {@link SupervisorResponseStrategy#LAST} 则直接返回最后一家 subagent 的输出，适合“最后一步就是交付物”的流程。
 * {@link SupervisorContextStrategy#SUMMARIZATION} 把各 subagent 的对话压成摘要再给主管；
 * 另外两个取值是只看主管自己的聊天记忆，或两者都给。
 */
public final class SupervisorExample {

    static final String SUPERVISOR_CONTEXT = """
            先了解订单事实和退货政策，两项都有了再调用 reply。
            reply 因为缺材料被跳过时，去补齐被点名的那项，不要重复调用同一家。
            不要编造物流状态。
            """;

    private SupervisorExample() {
    }

    public static SupervisorAgent create(ChatModel model) {
        return create(model, model);
    }

    /**
     * 规划模型和办事模型可以分开。主管负责选择和汇总，subagent 只做一件窄事，通常可以用更小的模型。
     */
    public static SupervisorAgent create(ChatModel plannerModel, ChatModel workerModel) {
        PolicyAgent policy = AgenticServices.agentBuilder(PolicyAgent.class)
                .chatModel(workerModel)
                .build();
        ReplyAgent reply = AgenticServices.agentBuilder(ReplyAgent.class)
                .chatModel(workerModel)
                .build();
        return crew(plannerModel, new OrderLookup(), policy, reply);
    }

    /**
     * handoff 客服台整体作为一个 subagent。主管只看到 {@code supportDesk} 这一个名字，
     * 台子内部的 triage → billing/tech → human 不会暴露给主管的规划循环。
     */
    public static SupervisorAgent createWithHandoffDesk(ChatModel plannerModel, SupportDesk desk) {
        return crew(plannerModel, desk, new OrderLookup());
    }

    private static SupervisorAgent crew(ChatModel plannerModel, Object... subAgents) {
        return AgenticServices.supervisorBuilder()
                .chatModel(plannerModel)
                .name("supportSupervisor")
                .description("客服主管，按需要查询订单、核对政策，并产出给用户的回复")
                .subAgents(subAgents)
                .supervisorContext(SUPERVISOR_CONTEXT)
                .contextGenerationStrategy(SupervisorContextStrategy.SUMMARIZATION)
                .responseStrategy(SupervisorResponseStrategy.SUMMARY)
                .maxAgentsInvocations(6)
                .build();
    }

    public interface PolicyAgent {
        @SystemMessage("你只解释退货政策：签收 7 天内且未拆封可以退。不要编造订单或物流。")
        @UserMessage("用户说：{{request}}")
        @Agent(name = "policy", description = "解释退货和退款政策，不查询订单", outputKey = "policyNote")
        String policy(@V("request") String request);
    }

    public interface ReplyAgent {
        @UserMessage("""
                写一段给用户的最终回复，不要提到内部代理。
                用户说：{{request}}
                订单事实：{{orderFacts}}
                政策说明：{{policyNote}}
                """)
        @Agent(name = "reply",
                description = "orderFacts 和 policyNote 都已经写好之后，撰写给用户的最终回复",
                outputKey = "reply",
                optional = true)
        String reply(@V("request") String request,
                     @V("orderFacts") String orderFacts,
                     @V("policyNote") String policyNote);
    }

    /** 非 AI 代理：一个带 {@code @Agent} 的普通方法，主管会把它和 LLM 专员同等看待。 */
    public static class OrderLookup {
        @Agent(name = "orderLookup",
                description = "用户提到订单、物流或发货时，查询订单事实",
                outputKey = "orderFacts")
        public String orderLookup(@V("request") String request) {
            if (request != null && request.contains("10086")) {
                return "订单 10086：已发货，预计明天送达。";
            }
            return "没有从请求中识别到订单号。";
        }
    }
}
