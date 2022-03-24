import React from "react";
import { useNavigate } from "react-router-dom";
import WztButton from '../wzt-button'
import logo from '../../images/github-icon.png'
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
                <a href="https://github.com/wangzt-arch"><img className="header-logo" src={logo} alt="" /></a>
                <WztButton text='Home' onNavigate={() => navigate('/myself/home')}></WztButton>
                <WztButton text='About' onNavigate={() => navigate('/myself/about')}></WztButton>
                <WztButton text='Docs' onNavigate={() => navigate('/myself/docs')}></WztButton>
            </div>
        </div>
    );
}

export default Header;
