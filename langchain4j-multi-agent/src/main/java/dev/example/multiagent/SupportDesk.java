package dev.example.multiagent;

import dev.langchain4j.agentic.Agent;
import dev.langchain4j.service.V;

/**
 * 对调用方暴露的唯一入口。分诊结论、上游说明都放在 {@code AgenticScope} 里，不出现在这个签名上，
 * 所以同一次用户请求可以先走过分诊、再走过专员，而 HTTP 或消息监听器只调用这一个方法。
 */
public interface SupportDesk {

    @Agent(
            name = "supportDesk",
            description = "客服入口。寒暄直接回答；账单和技术问题移交给对应专员，必要时再升级到人工。")
    String handle(@V("request") String request);
}
