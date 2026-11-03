# AI_Implementation
AI Searching and NLP

## LangChain4j 多 subagent 与 handoff

可运行案例在 [`langchain4j-multi-agent`](langchain4j-multi-agent)。架构写在 `dev.example.multiagent` 的包注释里，三种接法是：

- `SupervisorComposition.kt`：把 `supervisor.invoke()` 里面的循环摊开，看清规划模型和子代理各是一次调用
- `SupervisorExample`：主管持有控制权，按需多次调用 subagent
- `OneShotHandoffExample`：分类后把请求一次性移交给一个专员
- `ChainedHandoffExample`：专员用结构化结果把控制权交给下一个专员

```bash
cd langchain4j-multi-agent
mvn test
mvn -q exec:java
```
