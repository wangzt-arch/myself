import React, { useMemo, useState } from "react";
import { useLocation, useNavigate } from "react-router-dom";
import WztButton from "../wzt-button";
import MusicPlayer from "../MusicPlayer";
import logo from "../../images/github-icon.png";
import "./index.css";

const navItems = [
  { text: "首页", path: "/home" },
  { text: "关于", path: "/about" },
  { text: "文档", path: "/docs" },
  { text: "图表", path: "/chart" },
  { text: "翻译", path: "/translate" },
  { text: "虚拟列表", path: "/virtual-list" },
  { text: "三维场景", path: "/preview" },
  { text: "流程图", path: "/logicflow" },
  { text: "视频案例", path: "/video" },
  { text: "3D地球", path: "/cesium" },
];

function Header() {
  const navigate = useNavigate();
  const location = useLocation();
  const [menuOpen, setMenuOpen] = useState(false);

  const currentPath = useMemo(() => {
    return location.pathname === "/" || location.pathname === "/myself"
      ? "/home"
      : location.pathname;
  }, [location.pathname]);

  const navigateTo = (path) => {
    navigate(path);
    setMenuOpen(false);
  };

  return (
    <header className="header">
      <div className="header-brand-wrapper">
        <button
          className="header-brand"
          type="button"
          onClick={() => navigateTo("/home")}
        >
          <span className="brand-mark" aria-hidden="true">
            🌰
          </span>
          <span className="text-shadow">举个栗子</span>

        </button>
        <MusicPlayer />
      </div>

      <div className="header-actions">
        <a
          className="github-link"
          href="https://github.com/wangzt-arch"
          target="_blank"
          rel="noreferrer"
          aria-label="GitHub"
        >
          <img className="header-logo" src={logo} alt="" />
        </a>
        <button
          className={menuOpen ? "menu-toggle menu-toggle--open" : "menu-toggle"}
          type="button"
          aria-label="切换导航菜单"
          aria-expanded={menuOpen}
          onClick={() => setMenuOpen((value) => !value)}
        >
          <span></span>
          <span></span>
          <span></span>
        </button>
      </div>

      <nav
        className={menuOpen ? "header-nav header-nav--open" : "header-nav"}
        aria-label="主导航"
      >
        {navItems.map((item) => (
          <WztButton
            key={item.path}
            text={item.text}
            active={currentPath === item.path}
            onNavigate={() => navigateTo(item.path)}
          />
        ))}
      </nav>
    </header>
  );
}

export default Header;
