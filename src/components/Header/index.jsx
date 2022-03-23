import React from "react";
import { useNavigate } from "react-router-dom";
import "./index.css";

function Header() {
    const navigate = useNavigate();
    return (
        <div className="header">
            <div className="header-left" onClick={() => navigate('/myself/home')}>header</div>
            <div className="header-right" onClick={() => navigate("/myself/docs")}>
                To Docs
            </div>
        </div>
    );
}

export default Header;
