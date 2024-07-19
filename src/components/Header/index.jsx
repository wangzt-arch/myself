import React from "react";
import { useNavigate } from "react-router-dom";
import WztButton from "../wzt-button";
import logo from "../../images/github-icon.png";
import "./index.css";

function Header() {
  const navigate = useNavigate();
  return (
    <div className="header">
      <div className="header-left">
        {/* <div className="text-magic" data-word="WangZhiTao">
          WangZhiTao
          <div className="white"></div>
        </div> */}
        <div className="text-shadow">举个栗子 🌰</div>
      </div>
      <div className="header-right">
        <a href="https://github.com/wangzt-arch">
          <img className="header-logo" src={logo} alt="" />
        </a>
        <WztButton
          text="Home"
          onNavigate={() => navigate("/home")}
        ></WztButton>
        <WztButton
          text="About"
          onNavigate={() => navigate("/about")}
        ></WztButton>
        <WztButton
          text="Docs"
          onNavigate={() => navigate("/docs")}
        ></WztButton>
        <WztButton
          text="COVID-19"
          onNavigate={() => navigate("/yq")}
        ></WztButton>
        <WztButton
          text="翻译"
          onNavigate={() => navigate("/translate")}
        ></WztButton>
        <WztButton
          text="模型预览"
          onNavigate={() => navigate("/preview")}
        ></WztButton>
        <WztButton
          text="流程图"
          onNavigate={() => navigate("/logicflow")}
        ></WztButton>
      </div>
    </div>
  );
}

export default Header;
