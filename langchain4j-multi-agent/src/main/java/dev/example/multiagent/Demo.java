package dev.example.multiagent;

/**
 * 不访问模型，打印脚本专员的移交结果。
 *
 * <pre>
 *   cd langchain4j-multi-agent
 *   mvn -q exec:java
 * </pre>
 *
 * 换成真实模型时，编排代码就是下面这几行，密钥不要写进源码：
 *
 * <pre>
 *   ChatModel model = OpenAiModels.chatModel(System.getenv("OPENAI_API_KEY"), "gpt-4.1-mini");
 *   SupportDesk handoff = ChainedHandoffExample.withModel(model);
 *   SupportDesk oneShot = OneShotHandoffExample.withModel(model);
 *   SupervisorAgent crew = SupervisorExample.create(model);
 * </pre>
 */
public final class Demo {

    private Demo() {
    }

    public static void main(String[] args) {
        SupportDesk chained = ChainedHandoffExample.scripted();
        System.out.println(chained.handle("你好"));
        System.out.println(chained.handle("我要开发票"));
        System.out.println(chained.handle("登录失败，转人工"));

        SupportDesk oneShot = OneShotHandoffExample.scripted();
        System.out.println(oneShot.handle("我要开发票"));
        System.out.println(oneShot.handle("你好"));
    }
}
