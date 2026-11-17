package dev.example.multiagent;

/**
 * 专员交还给编排器的结构化结果。返回 record 时，LangChain4j 会要求模型产出同名字段的 JSON。
 *
 * <p>{@code nextAgent} 为 {@code done}（或空白）时，{@code answer} 是给用户的最终答复。
 * 否则 {@code answer} 只是写给下一个专员的移交说明，用户还看不到它。
 * 这样中间注记和最终答复共用一个类型，编排器不用再解析自由文本。
 *
 * @param answer    最终答复，或写给下一个专员的说明
 * @param nextAgent 下一个专员的名字，或 {@link #DONE}
 */
public record Handoff(String answer, String nextAgent) {

    /** 编排器认出这个标记后结束链路，并把 {@code answer} 交给用户。 */
    public static final String DONE = "done";

    public static Handoff done(String answer) {
        return new Handoff(answer, DONE);
    }

    public static Handoff to(String nextAgent, String noteForNextAgent) {
        return new Handoff(noteForNextAgent, nextAgent);
    }

    /** 模型漏了字段、填了空白，或显式写了 done，都视为这条链路结束。 */
    public boolean finished() {
        return nextAgent == null || nextAgent.isBlank() || DONE.equalsIgnoreCase(nextAgent.trim());
    }
}
