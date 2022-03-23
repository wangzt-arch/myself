import "./App.css";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import Home from "./pages/home";
import Docs from "./pages/docs";
import About from "./pages/about"

function App() {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/myself" element={<Home />} />
        <Route path="/myself/home" element={<Home />} />
        <Route path="/myself/docs" element={<Docs />} />
        <Route path="/myself/about" element={<About />} />
      </Routes>
    </BrowserRouter>
  );
}

export default App;
