import "./App.css";
import {BrowserRouter, Route, Routes} from "react-router-dom";
import Home from "./pages/home";
import Docs from "./pages/docs";
import About from "./pages/about";
import YqDistribution from "./pages/yq-distribution";

function App() {
    return (
        <BrowserRouter>
            <Routes>
                <Route path="/myself"
                    element={<Home/>}/>
                <Route path="/home"
                    element={<Home/>}/>
                <Route path="/home"
                    element={<Home/>}/>
                <Route path="/docs"
                    element={<Docs/>}/>
                <Route path="/about"
                    element={<About/>}/>
                <Route path="/yq"
                    element={<YqDistribution/>}/>
            </Routes>
        </BrowserRouter>
    );
}

export default App;
