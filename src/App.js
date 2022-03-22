import "./App.css";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import Home from "./pages/home";
import Docs from "./pages/docs";

function App() {
  return (
    <div>
      测试
      <BrowserRouter>
        <Routes>
          <Route path="/" element={<Home />} />
          <Route path="/home" element={<Home />} />
          <Route path="/docs" element={<Docs />} />
        </Routes>
      </BrowserRouter>
    </div>
  );
}

export default App;
