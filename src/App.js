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
          <Route path="/myself" element={<Home />} />
          <Route path="/myself/home" element={<Home />} />
          <Route path="/myself/docs" element={<Docs />} />
        </Routes>
      </BrowserRouter>
    </div>
  );
}

export default App;