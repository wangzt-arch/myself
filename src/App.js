import React, { Suspense, lazy, useEffect } from "react";
import { HashRouter, Route, Routes } from "react-router-dom";
import Header from "./components/Header";
import Home from "./pages/home";
import Docs from "./pages/docs";
import About from "./pages/about";
import Chart from "./pages/yq-distribution";
import NoFound from "./pages/no-found";
import Translate from "./pages/translate";
import LogicFlow from "./pages/logicflow";
import Video from "./pages/video";
import VirtualListPage from "./pages/virtual-list";
import OnlineTools from "./pages/online-tools";
import AIChat from "./pages/ai-chat";
import "./App.css";

const CesiumPage = lazy(() => import("./pages/cesium"));
const ModelPreview = lazy(() => import("./pages/model-preview"));

const preloadRoutes = [
  () => import("./pages/model-preview"),
  () => import("./pages/cesium"),
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
