# AI 圈子热词百科

> 本文档汇总了人工智能领域最常用、最热门的专业术语，帮助读者快速理解 AI 圈子的"黑话"。

---

## 一、基础概念类

### 1.1 大模型相关

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **大模型** | Large Language Model (LLM) | 参数量巨大（通常数十亿到数千亿）的神经网络模型，具备强大的语言理解和生成能力 |
| **GPT** | Generative Pre-trained Transformer | OpenAI 开发的生成式预训练 Transformer 模型系列，如 GPT-3、GPT-4 |
| **Transformer** | - | 一种基于自注意力机制的神经网络架构，是现代大模型的基础结构 |
| **参数** | Parameters | 模型中可学习的权重数值，参数量越大通常模型能力越强 |
| **Token** | - | 模型处理文本的最小单位，可以是一个字、一个词或一个词片段 |
| **上下文窗口** | Context Window | 模型一次能处理的 Token 数量上限，决定了模型能"记住"多少信息 |
| **幻觉** | Hallucination | 模型生成看似合理但实际上错误或不存在的虚假信息 |
| **涌现能力** | Emergent Ability | 模型规模达到某个临界点后才突然展现出的新能力 |

### 1.2 训练相关

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **预训练** | Pre-training | 在大规模无标注数据上训练模型，学习通用知识 |
| **微调** | Fine-tuning | 在预训练模型的基础上，用特定领域的小数据集进行针对性训练 |
| **RLHF** | Reinforcement Learning from Human Feedback | 基于人类反馈的强化学习，用于让模型输出更符合人类偏好 |
| **SFT** | Supervised Fine-Tuning | 监督微调，使用标注数据对模型进行训练 |
| **Loss** | - | 损失函数，衡量模型预测与真实值之间的差距 |
| **梯度下降** | Gradient Descent | 通过计算梯度来更新模型参数，使 Loss 最小化的优化算法 |
| **Epoch** | - | 训练过程中完整遍历一遍数据集的次数 |
| **Batch** | - | 一次前向传播和反向传播所使用的样本数量 |
| **过拟合** | Overfitting | 模型在训练数据上表现很好，但在新数据上表现差 |
| **欠拟合** | Underfitting | 模型过于简单，无法捕捉数据的复杂模式 |

---

## 二、技术架构类

### 2.1 模型架构

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **Attention** | 注意力机制 | 让模型在处理序列时能够关注不同位置的信息，是 Transformer 的核心 |
| **Self-Attention** | 自注意力 | 序列中的每个位置都能关注序列中其他所有位置 |
| **Multi-Head Attention** | 多头注意力 | 并行使用多组注意力机制，从不同角度捕捉信息 |
| **Embedding** | 嵌入 | 将离散的文本/图像等数据映射为连续的向量表示 |
| **Encoder** | 编码器 | 将输入数据编码为特征表示的网络组件 |
| **Decoder** | 解码器 | 将特征表示解码为输出的网络组件 |
| **MoE** | Mixture of Experts | 混合专家模型，通过多个专家网络提升模型容量和效率 |

### 2.2 推理与部署

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **推理** | Inference | 使用训练好的模型进行预测或生成的过程 |
| **Prompt** | 提示词 | 输入给模型的指令或问题，引导模型生成期望的输出 |
| **Prompt Engineering** | 提示工程 | 设计和优化 Prompt 以获得更好模型输出的技术 |
| **RAG** | Retrieval-Augmented Generation | 检索增强生成，结合外部知识库提升模型回答的准确性 |
| **量化** | Quantization | 将模型参数从高精度（如 FP32）转换为低精度（如 INT8），减少存储和计算需求 |
| **蒸馏** | Distillation | 用大模型（教师）训练小模型（学生），让小模型获得接近大模型的能力 |
| **KV Cache** | Key-Value Cache | 在自回归生成中缓存已计算的 Key 和 Value，加速推理 |
| **流式输出** | Streaming | 模型生成结果时分段返回，而不是等全部生成完再返回 |

---

## 三、应用场景类

### 3.1 生成式 AI

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **AIGC** | AI Generated Content | 人工智能生成内容，包括文本、图像、音频、视频等 |
| **文生图** | Text-to-Image | 根据文本描述生成图像，如 Midjourney、Stable Diffusion |
| **文生视频** | Text-to-Video | 根据文本描述生成视频，如 Sora、Runway |
| **多模态** | Multimodal | 能够同时处理和理解多种类型数据（文本、图像、音频等）的模型 |
| **Copilot** | 副驾驶/助手 | 辅助人类完成任务的 AI 工具，如 GitHub Copilot |
| **Agent** | 智能体 | 能够自主感知环境、做出决策并执行动作的 AI 系统 |
| **RPA** | Robotic Process Automation | 机器人流程自动化，用软件机器人模拟人类操作 |
| **Skill** | 技能/工具 | AI 系统中可复用的功能模块，如 MCP Skill、Function Calling 中的工具调用 |

### 3.2 行业应用

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **Chatbot** | 聊天机器人 | 能够与人类进行自然语言对话的 AI 程序 |
| **数字人** | Digital Human | 基于 AI 生成的虚拟人物形象，可交互、可驱动 |
| **智能客服** | - | 使用 AI 技术自动回答客户咨询的系统 |
| **代码生成** | Code Generation | AI 根据自然语言描述或上下文自动生成代码 |
| **知识图谱** | Knowledge Graph | 用图结构表示实体及其关系的知识库 |
| **OCR** | Optical Character Recognition | 光学字符识别，将图像中的文字转换为可编辑文本 |
| **NLP** | Natural Language Processing | 自然语言处理，让计算机理解人类语言的技术 |
| **CV** | Computer Vision | 计算机视觉，让计算机"看懂"图像和视频的技术 |
| **ASR** | Automatic Speech Recognition | 自动语音识别，将语音转换为文字 |
| **TTS** | Text-to-Speech | 文本转语音，将文字转换为可播放的语音 |

---

## 四、行业生态类

### 4.1 公司与产品

| 热词 | 说明 |
|------|------|
| **OpenAI** | 开发了 GPT 系列和 ChatGPT 的 AI 研究公司 |
| **Anthropic** | 开发了 Claude 系列模型的 AI 安全公司 |
| **Google DeepMind** | Google 旗下的 AI 研究部门，开发了 Gemini |
| **Meta AI** | Meta（Facebook）的 AI 研究团队，开发了 LLaMA |
| **Midjourney** | 知名的 AI 图像生成工具 |
| **Stable Diffusion** | 开源的 AI 图像生成模型 |
| **Hugging Face** | 全球最大的 AI 模型和数据集开源社区 |
| **LangChain** | 用于构建 LLM 应用的开发框架 |

### 4.2 开源与社区

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **开源模型** | Open Source Model | 模型权重和代码公开，可自由使用和修改 |
| **闭源模型** | Closed Source Model | 模型不公开，只能通过 API 调用，如 GPT-4 |
| **API** | Application Programming Interface | 应用程序接口，调用模型服务的标准方式 |
| **SDK** | Software Development Kit | 软件开发工具包，方便开发者集成 AI 能力 |
| **GitHub** | - | 全球最大的代码托管平台，AI 开源项目的主要聚集地 |
| **Hugging Face Hub** | - | 托管和分享 AI 模型、数据集的平台 |
| **模型卡** | Model Card | 记录模型信息、用途、限制等元数据的文档 |
| **数据卡** | Dataset Card | 记录数据集来源、构成、使用方式等信息的文档 |

---

## 五、前沿趋势类

### 5.1 新兴技术

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **AGI** | Artificial General Intelligence | 通用人工智能，具备人类水平综合智能的 AI 系统 |
| **ASI** | Artificial Super Intelligence | 超级人工智能，超越人类智能的 AI 系统 |
| **具身智能** | Embodied AI | 具有物理身体、能与真实世界交互的 AI |
| **世界模型** | World Model | 能够理解和模拟物理世界运行规律的 AI 模型 |
| **神经符号** | Neuro-symbolic AI | 结合神经网络和符号推理的混合 AI 方法 |
| **联邦学习** | Federated Learning | 在保护数据隐私的前提下，分布式训练模型的技术 |
| **边缘 AI** | Edge AI | 在终端设备（手机、摄像头等）上运行的 AI |

### 5.2 安全与伦理

| 热词 | 英文/缩写 | 含义解释 |
|------|-----------|----------|
| **AI 安全** | AI Safety | 研究如何确保 AI 系统安全、可靠、可控 |
| **AI 对齐** | AI Alignment | 确保 AI 的目标和行为与人类价值观一致 |
| **红队测试** | Red Teaming | 模拟攻击者角色，测试 AI 系统的漏洞和风险 |
| **越狱** | Jailbreak | 通过特定 Prompt 绕过模型的安全限制 |
| **提示注入** | Prompt Injection | 在输入中插入恶意指令，操纵模型行为 |
| **数据隐私** | Data Privacy | 保护训练数据和个人信息不被滥用 |
| **算法偏见** | Algorithmic Bias | AI 模型因训练数据问题而产生的歧视性输出 |
| **可解释性** | Explainability | 让 AI 的决策过程对人类可理解 |

---

## 六、常用缩略语速查

| 缩写 | 全称 | 中文 |
|------|------|------|
| AI | Artificial Intelligence | 人工智能 |
| ML | Machine Learning | 机器学习 |
| DL | Deep Learning | 深度学习 |
| LLM | Large Language Model | 大语言模型 |
| NLP | Natural Language Processing | 自然语言处理 |
| CV | Computer Vision | 计算机视觉 |
| ASR | Automatic Speech Recognition | 自动语音识别 |
| TTS | Text-to-Speech | 文本转语音 |
| OCR | Optical Character Recognition | 光学字符识别 |
| API | Application Programming Interface | 应用程序接口 |
| GPU | Graphics Processing Unit | 图形处理器 |
| TPU | Tensor Processing Unit | 张量处理器 |
| SOTA | State of the Art | 当前最优 |
| ROI | Return on Investment | 投资回报率 |
| POC | Proof of Concept | 概念验证 |
| MVP | Minimum Viable Product | 最小可行产品 |

---

> **更新说明**：本文档持续更新中，如有遗漏或错误，欢迎补充指正。
