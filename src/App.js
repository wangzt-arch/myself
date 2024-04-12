import "./App.css";
import { HashRouter, Route, Routes } from "react-router-dom";
import Home from "./pages/home";
import Docs from "./pages/docs";
import About from "./pages/about";
import YqDistribution from "./pages/yq-distribution";
import NoFound from "./pages/no-found";
import Translate from "./pages/translate"
import ModelPreview from "./pages/model-preview"

function App() {
    return (
        <HashRouter>
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
                <Route path="/yq"
                    element={<YqDistribution />} />
                <Route path="/translate"
                    element={<Translate />} />
                <Route path="/preview"
                    element={<ModelPreview />} />
                <Route path="*"
                    element={<NoFound />} />
            </Routes>
        </HashRouter>
    );
}

export default App;
