package dev.example.multiagent;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import dev.langchain4j.agentic.Agent;
import dev.langchain4j.model.chat.ChatModel;
import dev.langchain4j.model.chat.request.ChatRequest;
import dev.langchain4j.model.chat.response.ChatResponse;
import dev.langchain4j.service.V;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class HandoffBehaviorTest {

    @Test
    @DisplayName("寒暄在分诊结束，不进入专员")
    void greetingStopsAtTriage() {
        SupportDesk desk = ChainedHandoffExample.scripted();

        String first = desk.handle("你好");
        String second = desk.handle("你好");

        assertEquals("分诊：可以直接答复。你好，我是客服入口。", first);
        assertEquals(first, second);
    }

    @Test
    @DisplayName("发票请求移交给账单专员，并由账单专员结束")
    void invoiceHandsOffToBilling() {
        String answer = ChainedHandoffExample.scripted().handle("我要开发票");

        assertEquals("账单专员：发票将在 1 个工作日内发送到预留邮箱。", answer);
    }

    @Test
    @DisplayName("技术问题再要求人工时，控制权从 tech 交给 human")
    void loginEscalatesToHuman() {
        String answer = ChainedHandoffExample.scripted().handle("登录失败，转人工");

        assertTrue(answer.contains("人工坐席已接手"));
        assertTrue(answer.contains("重置密码"));
        assertFalse(answer.contains("账单专员"));
    }

    @Test
    @DisplayName("分诊不允许直达人工，越权点名被忽略")
    void disallowedHandoffIsDropped() {
        SupportDesk desk = ChainedHandoffExample.desk(
                "triage",
                4,
                ChainedHandoffExample.SUPPORT_GRAPH,
                new JumpToHuman(),
                new ChainedHandoffExample.ScriptedBilling(),
                new ChainedHandoffExample.ScriptedTech(),
                new ChainedHandoffExample.ScriptedHuman());

        String answer = desk.handle("随便什么问题");

        assertEquals("分诊越权", answer);
    }

    @Test
    @DisplayName("点名不存在的专员时返回当前说明")
    void unknownTargetEndsTheChain() {
        SupportDesk desk = ChainedHandoffExample.desk(
                "triage",
                4,
                ChainedHandoffExample.SUPPORT_GRAPH,
                new JumpToGhost(),
                new ChainedHandoffExample.ScriptedBilling(),
                new ChainedHandoffExample.ScriptedTech(),
                new ChainedHandoffExample.ScriptedHuman());

        assertEquals("没有这家专员", desk.handle("随便什么问题"));
    }

    @Test
    @DisplayName("互相移交在跳数上限处停下")
    void pingPongStopsAtMaxInvocations() {
        Map<String, Set<String>> loop = Map.of("a", Set.of("b"), "b", Set.of("a"));

        String capped = ChainedHandoffExample.desk("a", 3, loop, new PingA(), new PingB()).handle("ping");
        String single = ChainedHandoffExample.desk("a", 1, loop, new PingA(), new PingB()).handle("ping");

        assertEquals("from-a", capped);
        assertEquals("from-a", single);
    }

    @Test
    @DisplayName("一次移交只启动类别对应的专员")
    void oneShotRoutesToASingleSpecialist() {
        SupportDesk desk = OneShotHandoffExample.scripted();

        assertEquals("账单专员：已登记发票申请。", desk.handle("我要开发票"));
        assertEquals("技术专员：请重置密码后重新登录。", desk.handle("登录失败"));
        assertEquals("通用客服：请补充订单号或报错原文。", desk.handle("你好"));
    }

    @Test
    @DisplayName("主管图和嵌套 handoff 可以在不调用模型的情况下装配完成")
    void supervisorGraphBuildsWithoutCallingTheModel() {
        ChatModel unused = new ChatModel() {
            @Override
            public ChatResponse doChat(ChatRequest chatRequest) {
                throw new UnsupportedOperationException("this test does not call a model");
            }
        };

        SupervisorExample.create(unused);
        SupervisorExample.createWithHandoffDesk(unused, ChainedHandoffExample.scripted());
        ChainedHandoffExample.withModel(unused);
        OneShotHandoffExample.withModel(unused);
    }

    public static class JumpToHuman {
        @Agent(name = "triage", description = "试图越过允许列表直接转人工", outputKey = "handoff")
        public Handoff triage(@V("request") String request) {
            return Handoff.to("human", "分诊越权");
        }
    }

    public static class JumpToGhost {
        @Agent(name = "triage", description = "点名一个不存在的专员", outputKey = "handoff")
        public Handoff triage(@V("request") String request) {
            return Handoff.to("ghost", "没有这家专员");
        }
    }

    public static class PingA {
        @Agent(name = "a", description = "总是交给 b", outputKey = "handoff")
        public Handoff a(@V("request") String request) {
            return Handoff.to("b", "from-a");
        }
    }

    public static class PingB {
        @Agent(name = "b", description = "总是交给 a", outputKey = "handoff")
        public Handoff b(@V("request") String request) {
            return Handoff.to("a", "from-b");
        }
    }
}
