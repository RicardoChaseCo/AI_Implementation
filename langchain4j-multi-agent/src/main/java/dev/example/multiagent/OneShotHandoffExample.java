package dev.example.multiagent;

import dev.langchain4j.agentic.Agent;
import dev.langchain4j.agentic.AgenticServices;
import dev.langchain4j.model.chat.ChatModel;
import dev.langchain4j.service.SystemMessage;
import dev.langchain4j.service.UserMessage;
import dev.langchain4j.service.V;

/**
 * 一次移交。分类器把类别写进 scope，条件工作流只启动匹配的那一个专员，专员写好的 {@code answer} 就是最终结果。
 *
 * <p>条件工作流在第一轮把所有命中的专员一起调用，所以三个谓词做成互斥：同一条请求只会命中一家。
 * 深度固定为 1，专员不能再把请求转走。需要升级链路时用 {@link ChainedHandoffExample}。
 *
 * <p>分类器返回枚举而不是自由文本，合法类别由类型收口。有 {@code DecisionModel} 时，官方的
 * {@code DecisionRouterPlanner}（在 {@code langchain4j-agentic-patterns}）可以去掉这个分类器，
 * 直接按各专员的 {@code description} 打分选择；本案例不引入那份额外依赖。
 */
public final class OneShotHandoffExample {

    private OneShotHandoffExample() {
    }

    public enum Specialist {
        billing, tech, general
    }

    public static SupportDesk scripted() {
        return desk(new ScriptedRouter(), new ScriptedBilling(), new ScriptedTech(), new ScriptedGeneral());
    }

    public static SupportDesk withModel(ChatModel model) {
        return desk(
                AgenticServices.agentBuilder(CategoryRouter.class).chatModel(model).build(),
                AgenticServices.agentBuilder(BillingExpert.class).chatModel(model).build(),
                AgenticServices.agentBuilder(TechExpert.class).chatModel(model).build(),
                AgenticServices.agentBuilder(GeneralExpert.class).chatModel(model).build());
    }

    public static SupportDesk desk(Object router, Object billing, Object tech, Object general) {
        // 条件代理和后面的专员读的是同一块 scope，所以路由结果用 outputKey "category" 交接。
        Object specialist = AgenticServices.conditionalBuilder()
                .name("specialistHandoff")
                .subAgents("category is billing",
                        scope -> scope.readState("category", Specialist.general) == Specialist.billing,
                        billing)
                .subAgents("category is tech",
                        scope -> scope.readState("category", Specialist.general) == Specialist.tech,
                        tech)
                .subAgents("category is general",
                        scope -> scope.readState("category", Specialist.general) == Specialist.general,
                        general)
                .build();

        return AgenticServices.sequenceBuilder(SupportDesk.class)
                .name("oneShotHandoff")
                .subAgents(router, specialist)
                .outputKey("answer")
                .build();
    }

    public interface CategoryRouter {
        @SystemMessage("""
                把用户请求分成且仅分成一个类别：billing、tech、general。
                billing：发票、退款、账单、扣费。
                tech：登录、密码、报错、无法使用。
                其余是 general。
                """)
        @UserMessage("用户说：{{request}}")
        @Agent(name = "router", description = "把请求分成 billing、tech 或 general", outputKey = "category")
        Specialist route(@V("request") String request);
    }

    public interface BillingExpert {
        @UserMessage("你是账单专员。用两三句话回答：{{request}}")
        @Agent(name = "billing", description = "回答发票、退款和账单", outputKey = "answer")
        String answer(@V("request") String request);
    }

    public interface TechExpert {
        @UserMessage("你是技术专员。用两三句话给出操作步骤：{{request}}")
        @Agent(name = "tech", description = "回答登录、密码和故障", outputKey = "answer")
        String answer(@V("request") String request);
    }

    public interface GeneralExpert {
        @UserMessage("你是通用客服。用户说：{{request}}。请其补充订单号或报错原文。")
        @Agent(name = "general", description = "回答无法归入账单或技术的请求", outputKey = "answer")
        String answer(@V("request") String request);
    }

    public static class ScriptedRouter {
        @Agent(name = "router", description = "按关键词写入类别", outputKey = "category")
        public Specialist route(@V("request") String request) {
            String text = request == null ? "" : request;
            if (text.contains("发票") || text.contains("账单") || text.contains("退款")) {
                return Specialist.billing;
            }
            if (text.contains("登录") || text.contains("密码") || text.contains("报错")) {
                return Specialist.tech;
            }
            return Specialist.general;
        }
    }

    public static class ScriptedBilling {
        @Agent(name = "billing", description = "账单答复", outputKey = "answer")
        public String answer(@V("request") String request) {
            return "账单专员：已登记发票申请。";
        }
    }

    public static class ScriptedTech {
        @Agent(name = "tech", description = "技术答复", outputKey = "answer")
        public String answer(@V("request") String request) {
            return "技术专员：请重置密码后重新登录。";
        }
    }

    public static class ScriptedGeneral {
        @Agent(name = "general", description = "通用答复", outputKey = "answer")
        public String answer(@V("request") String request) {
            return "通用客服：请补充订单号或报错原文。";
        }
    }
}
