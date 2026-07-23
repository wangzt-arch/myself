import React from "react";
import { useNavigate } from "react-router-dom";

import MessageBoard from "../../components/MessageBoard";
import useScrollReveal from "../../hooks/useScrollReveal";
import "./index.css";

const featureItems = [
    {
        title: "三维场景",
        path: "/preview",
        category: "Three.js",
        description: "基于 React Three，支持本地模型加载和三维智慧园区场景。",
    },
    {
        title: "流程图实验室",
        path: "/logicflow",
        category: "LogicFlow",
        description: "沉淀流程编排、节点样式、导出能力，用来验证可视化编辑器方案。",
    },
    {
        title: "技术文档",
        path: "/docs",
        category: "Markdown",
        description: "把常用工程知识整理成可阅读的笔记，包含代码高亮和主题分类。",
    },
    {
        title: "视频案例",
        path: "/video",
        category: "Media",
        description: "集中展示视频素材与前端播放器布局，作为动效与媒体展示样例。",
    },
    {
        title: "图表",
        path: "/chart",
        category: "ECharts",
        description: "基于 ECharts 的数据可视化图表模块。",
    },
    {
        title: "3D 地球",
        path: "/cesium",
        category: "CesiumJS",
        description: "CesiumJS 驱动的 3D 地球，支持卫星影像、行政区划和城市标记交互。",
    },
    {
        title: "AI 对话",
        path: "/ai-chat",
        category: "Streaming",
        description: "模拟 AI 流式响应的对话界面，支持 Markdown 实时渲染与打字机效果。",
    },
    {
        title: "在线工具",
        path: "/online-tools",
        category: "Utilities",
        description: "收录 JSON 格式化、正则测试、时间戳转换等前端常用小工具。",
    },
    {
        title: "虚拟列表",
        path: "/virtual-list",
        category: "Performance",
        description: "大数据量场景下的虚拟滚动方案，含分组折叠与搜索过滤。",
    },
    {
        title: "翻译",
        path: "/translate",
        category: "i18n",
        description: "多语言文本翻译与对照查看，用于验证前端国际化方案。",
    },
    {
        title: "AI 生图生视频",
        externalUrl: "https://wangzt-arch.github.io/wztagent/",
        category: "AIGC",
        description: "基于 AI 的文生图、文生视频创作工具，支持多种风格与参数调整。",
    },
];

const skillItems = [
    "React",
    "Webpack",
    "Three.js",
    "LogicFlow",
    "Markdown",
    "ECharts",
    "PDF 预览",
    "响应式布局",
    "Cesium",
    "Vue",
];

const skillUrls = {
    React: "https://react.dev",
    Webpack: "https://webpack.js.org",
    "Three.js": "https://threejs.org",
    LogicFlow: "https://logic-flow.cn",
    Markdown: "https://www.markdownguide.org",
    ECharts: "https://echarts.apache.org",
    Cesium: "https://cesium.com/platform/cesiumjs",
    Vue: "https://vuejs.org",
};

const heroRoutes = [
    {
        label: "Three",
        title: "三维智慧园区",
        description: "进入可交互的模型与场景预览。",
        path: "/preview",
    },
    {
        label: "Docs",
        title: "技术文档与笔记",
        description: "阅读工程实践、问题拆解和实现记录。",
        path: "/docs",
    },
    {
        label: "AIGC",
        title: "AI 生图生视频",
        description: "用自然语言创作图像与视频。",
        externalUrl: "https://wangzt-arch.github.io/wztagent/",
    },
    {
        label: "Earth",
        title: "3D 地球",
        description: "CesiumJS 驱动的 3D 地球，支持卫星影像、行政区划和城市标记交互。",
        path: "/cesium",
    },
];

function Home() {
    const navigate = useNavigate();

    const scrollToFeatured = () => {
        document.getElementById("featured-works")?.scrollIntoView({
            behavior: "smooth",
            block: "start",
        });
    };

    // 各区块的滚动入场动画
    const heroReveal = useScrollReveal({ threshold: 0.1 });
    const featureReveal = useScrollReveal();
    const skillReveal = useScrollReveal();

    return (
        <div className="home">

            <main className="home-main">
                <section
                    className={`home-hero reveal ${heroReveal.isVisible ? "reveal--visible" : ""}`}
                    ref={heroReveal.ref}
                >
                    <div className="hero-intro">
                        <div className="hero-overline">
                            <span className="hero-status-dot" aria-hidden="true"></span>
                            <span>FRONTEND LAB</span>
                            <span className="hero-overline-muted">持续更新中</span>
                        </div>
                        <p className="hero-kicker">个人技术实验室</p>
                        <h1>前端实验，做成可以打开的作品。</h1>
                        <p className="hero-summary">
                            React 工程实践、3D 可视化、AI 交互和技术笔记，集中在一个可以直接体验的实验室里。
                        </p>
                        <div className="hero-actions">
                            <button className="hero-button hero-button--primary" type="button" onClick={scrollToFeatured}>
                                查看精选项目 <span aria-hidden="true">↓</span>
                            </button>
                        </div>
                        <div className="hero-stats" aria-label="实验室内容统计">
                            <div><strong>11</strong><span>个作品</span></div>
                            <div><strong>21</strong><span>篇笔记</span></div>
                        </div>
                    </div>

                    <div className="hero-showcase-column">
                        <div className="hero-route-list" aria-label="实验室入口">
                            <p className="hero-route-heading">继续探索</p>
                            {heroRoutes.map((route) => (
                                route.externalUrl ? (
                                    <a
                                        className="hero-route"
                                        key={route.externalUrl}
                                        href={route.externalUrl}
                                        target="_blank"
                                        rel="noopener noreferrer"
                                    >
                                        <span className="hero-route-label">{route.label}</span>
                                        <span className="hero-route-copy">
                                            <strong>{route.title}</strong>
                                            <span>{route.description}</span>
                                        </span>
                                        <span className="hero-route-arrow" aria-hidden="true">→</span>
                                    </a>
                                ) : (
                                    <button
                                        className="hero-route"
                                        key={route.path}
                                        type="button"
                                        onClick={() => navigate(route.path)}
                                    >
                                        <span className="hero-route-label">{route.label}</span>
                                        <span className="hero-route-copy">
                                            <strong>{route.title}</strong>
                                            <span>{route.description}</span>
                                        </span>
                                        <span className="hero-route-arrow" aria-hidden="true">→</span>
                                    </button>
                                )
                            ))}
                        </div>
                    </div>
                </section>

                <section
                    id="featured-works"
                    className={`home-section reveal ${featureReveal.isVisible ? "reveal--visible" : ""}`}
                    ref={featureReveal.ref}
                >
                    <div className="section-heading">
                        <p>精选入口</p>
                        <h2>从已有页面里整理出的作品线索</h2>
                    </div>
                    <div className="feature-grid">
                        {featureItems.map((item, index) => (
                            item.externalUrl ? (
                                <a
                                    className={`feature-card reveal reveal--delay-${index + 1} ${featureReveal.isVisible ? "reveal--visible" : ""}`}
                                    key={item.externalUrl}
                                    href={item.externalUrl}
                                    target="_blank"
                                    rel="noopener noreferrer"
                                >
                                    <span className="feature-category">{item.category}</span>
                                    <strong>{item.title}</strong>
                                    <span>{item.description}</span>
                                </a>
                            ) : (
                                <button
                                    className={`feature-card reveal reveal--delay-${index + 1} ${featureReveal.isVisible ? "reveal--visible" : ""}`}
                                    key={item.path}
                                    type="button"
                                    onClick={() => navigate(item.path)}
                                >
                                    <span className="feature-category">{item.category}</span>
                                    <strong>{item.title}</strong>
                                    <span>{item.description}</span>
                                </button>
                            )
                        ))}
                    </div>
                </section>

                <section
                    className={`home-section home-section--skills reveal ${skillReveal.isVisible ? "reveal--visible" : ""}`}
                    ref={skillReveal.ref}
                >
                    <div className="section-heading">
                        <p>能力拼图</p>
                        <h2>这个站点正在积累的技术面</h2>
                    </div>
                    <div className="skill-list">
                        {skillItems.map((item, index) => (
                            <a
                                href={skillUrls[item]}
                                target="_blank"
                                rel="noopener noreferrer"
                                title={`访问 ${item} 官网`}
                                key={item}
                                className={`reveal reveal--delay-${index + 1} ${skillReveal.isVisible ? "reveal--visible" : ""}`}
                            >
                                {item}
                            </a>
                        ))}
                    </div>
                </section>
            </main>
            <MessageBoard />
        </div>
    );
}

export default Home;
