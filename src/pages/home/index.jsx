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

function Home() {
    const navigate = useNavigate();

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
                    <div className="hero-copy">
                        <p className="hero-kicker">个人技术实验室</p>
                        <h1>把前端想法做成可以打开、可以操作、可以复盘的作品。</h1>
                        <p className="hero-summary">
                            这里收纳 React 工程实践、3D 可视化、流程图编辑、技术文档和媒体案例。首页会作为作品导航，
                            帮你快速进入最值得看的内容。
                        </p>
                        <div className="hero-actions">
                            <button className="hero-button hero-button--primary" type="button" onClick={() => navigate("/preview")}>
                                查看 3D 作品
                            </button>
                            <button className="hero-button" type="button" onClick={() => navigate("/docs")}>
                                阅读技术笔记
                            </button>
                        </div>
                    </div>

                    <div className="hero-panel" aria-label="站点概览">
                        <div className="panel-row">
                            <span>当前重点</span>
                            <strong>作品集首页改造</strong>
                        </div>
                        <div className="panel-row">
                            <span>内容方向</span>
                            <strong>工具、文档、可视化</strong>
                        </div>
                        <div className="panel-meter">
                            <span style={{ width: "72%" }}></span>
                        </div>
                        <p>下一步可以继续完善项目详情、移动端导航和主题装饰。</p>
                    </div>
                </section>

                <section
                    className={`home-section reveal ${featureReveal.isVisible ? "reveal--visible" : ""}`}
                    ref={featureReveal.ref}
                >
                    <div className="section-heading">
                        <p>精选入口</p>
                        <h2>从已有页面里整理出的作品线索</h2>
                    </div>
                    <div className="feature-grid">
                        {featureItems.map((item, index) => (
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
