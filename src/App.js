import "./App.css";
import { HashRouter, Route, Routes } from "react-router-dom";
import Header from "./components/Header";
import Home from "./pages/home";
import Docs from "./pages/docs";
import About from "./pages/about";
import Chart from "./pages/yq-distribution";
import NoFound from "./pages/no-found";
import Translate from "./pages/translate"
import ModelPreview from "./pages/model-preview"
import LogicFlow from "./pages/logicflow"
import Video from "./pages/video"
import CesiumPage from "./pages/cesium"
import VirtualListPage from "./pages/virtual-list"
import OnlineTools from "./pages/online-tools"

function App() {
    return (
        <HashRouter
            future={{
                v7_startTransition: true,
                v7_relativeSplatPath: true,
            }}
        >
            <Header />
            <div className="app-content">
                <Routes>
                    <Route path="/"
                        element={<Home />} />
                    <Route path="/myself"
                        element={<Home />} />
                    <Route path="/home"
                        element={<Home />} />
                    <Route path="/docs"
                        element={<Docs />} />
                    <Route path="/about"
                        element={<About />} />
                    <Route path="/chart"
                        element={<Chart />} />
                    <Route path="/translate"
                        element={<Translate />} />
                    <Route path="/preview"
                        element={<ModelPreview />} />
                    <Route path="/logicflow"
                        element={<LogicFlow />} />
                    <Route path="/video"
                        element={<Video />} />
                    <Route path="/cesium"
                        element={<CesiumPage />} />
                    <Route path="/virtual-list"
                        element={<VirtualListPage />} />
                    <Route path="/online-tools"
                        element={<OnlineTools />} />
                    <Route path="*"
                        element={<NoFound />} />
                </Routes>
            </div>
        </HashRouter>
    );
}

export default App;
