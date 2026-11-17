/**
 * LangChain4j 1.23 里，多个代理协作走 {@code langchain4j-agentic}，而不是把几个 {@code AiServices} 互相注入。
 *
 * <p>同一次调用里的代理共用一块 {@link dev.langchain4j.agentic.scope.AgenticScope}。
 * 每个代理把返回值写到自己的 {@code outputKey}，下游用方法参数上的 {@code @V("键")} 来读。
 * 代理不持有彼此的 Java 引用，换谁先谁后只改编排，不改专员本身。
 *
 * <p>三种控制权，对应三个案例：
 *
 * <ul>
 *   <li>{@link dev.example.multiagent.SupervisorExample}
 *       主管模型每轮挑选一个或多个 subagent，拿到结果后控制权回到主管，由主管决定还要不要继续。
 *       步骤数事先不知道、中途要回头补材料时用这种。</li>
 *   <li>{@link dev.example.multiagent.OneShotHandoffExample}
 *       分类器先把类别写进 scope，条件工作流再把请求移交给唯一的专员，专员的答复就是最终答复。
 *       一次分流、没有二次升级时用这种。</li>
 *   <li>{@link dev.example.multiagent.ChainedHandoffExample}
 *       当前专员返回 {@link dev.example.multiagent.Handoff}，
 *       {@link dev.example.multiagent.HandoffPlanner} 按允许列表把控制权交给下一个专员，之后不再询问上一个。
 *       客服升级这种“移交之后由接收方负责”的流程用这种。</li>
 * </ul>
 *
 * <p>主管模式和 handoff 可以叠在一起：把整份 {@code SupportDesk} 当作主管的一个 subagent。
 * 主管只看见一个“客服入口”，入口内部如何分诊对主管是黑盒。见
 * {@link dev.example.multiagent.SupervisorExample#createWithHandoffDesk}。
 *
 * <p>{@code langchain4j-agentic} 仍是实验 API。本案例对齐 {@code 1.23.0-beta33}。
 * 不需要模型密钥的脚本路径可以用 {@code mvn test} 或 {@code mvn exec:java} 跑通。
 */
package dev.example.multiagent;
