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
        <div className="text-magic" data-word="WangZhiTao">
          WangZhiTao
          <div className="white"></div>
        </div>
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
          text="疫情"
          onNavigate={() => navigate("/yq")}
        ></WztButton>
      </div>
    </div>
  );
}

export default Header;
