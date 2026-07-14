import React, { Suspense, lazy, useEffect } from "react";
import { HashRouter, Route, Routes } from "react-router-dom";
import Header from "./components/Header";
import "./App.css";

const Home = lazy(() => import("./pages/home"));
const Docs = lazy(() => import("./pages/docs"));
const About = lazy(() => import("./pages/about"));
const Chart = lazy(() => import("./pages/yq-distribution"));
const NoFound = lazy(() => import("./pages/no-found"));
const Translate = lazy(() => import("./pages/translate"));
const ModelPreview = lazy(() => import("./pages/model-preview"));
const LogicFlow = lazy(() => import("./pages/logicflow"));
const Video = lazy(() => import("./pages/video"));
const CesiumPage = lazy(() => import("./pages/cesium"));
const VirtualListPage = lazy(() => import("./pages/virtual-list"));
const OnlineTools = lazy(() => import("./pages/online-tools"));
const AIChat = lazy(() => import("./pages/ai-chat"));

const preloadRoutes = [
  () => import("./pages/docs"),
  () => import("./pages/about"),
  () => import("./pages/yq-distribution"),
  () => import("./pages/translate"),
  () => import("./pages/model-preview"),
  () => import("./pages/logicflow"),
  () => import("./pages/video"),
  () => import("./pages/cesium"),
  () => import("./pages/virtual-list"),
  () => import("./pages/online-tools"),
  () => import("./pages/ai-chat"),
];

function PageLoader() {
  return (
    <div className="page-loader">
      <div className="page-loader__spinner"></div>
      <span className="page-loader__text">加载中...</span>
    </div>
  );
}

function App() {
  useEffect(() => {
    if ("requestIdleCallback" in window) {
      requestIdleCallback(() => {
        preloadRoutes.forEach((preload) => preload().catch(() => {}));
      });
    } else {
      const timer = setTimeout(() => {
        preloadRoutes.forEach((preload) => preload().catch(() => {}));
      }, 1500);
      return () => clearTimeout(timer);
    }
  }, []);

  return (
    <HashRouter
      future={{
        v7_startTransition: true,
        v7_relativeSplatPath: true,
      }}
    >
      <Header />
      <div className="app-content">
        <Suspense fallback={<PageLoader />}>
          <Routes>
            <Route path="/" element={<Home />} />
            <Route path="/myself" element={<Home />} />
            <Route path="/home" element={<Home />} />
            <Route path="/docs" element={<Docs />} />
            <Route path="/about" element={<About />} />
            <Route path="/chart" element={<Chart />} />
            <Route path="/translate" element={<Translate />} />
            <Route path="/preview" element={<ModelPreview />} />
            <Route path="/logicflow" element={<LogicFlow />} />
            <Route path="/video" element={<Video />} />
            <Route path="/cesium" element={<CesiumPage />} />
            <Route path="/virtual-list" element={<VirtualListPage />} />
            <Route path="/online-tools" element={<OnlineTools />} />
            <Route path="/ai-chat" element={<AIChat />} />
            <Route path="*" element={<NoFound />} />
          </Routes>
        </Suspense>
      </div>
    </HashRouter>
  );
}

export default App;
