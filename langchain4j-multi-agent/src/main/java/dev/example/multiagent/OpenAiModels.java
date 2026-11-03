package dev.example.multiagent;

import dev.langchain4j.model.chat.ChatModel;
import dev.langchain4j.model.openai.OpenAiChatModel;

/** 案例使用的模型工厂。任何 {@link ChatModel} 都能传给各个 Example，这里只是 OpenAI 兼容端点的一种接法。 */
public final class OpenAiModels {

    private OpenAiModels() {
    }

    public static ChatModel chatModel(String apiKey, String modelName) {
        return OpenAiChatModel.builder()
                .apiKey(apiKey)
                .modelName(modelName)
                .build();
    }
}
