import React from "react";
import Header from "../../components/Header";
import { useEffect } from "react";
import "./index.css";

function Home() {
    useEffect(() => {
        console.log('s');
        // getTranslate('word')
    }, [])
    return (
        <div className="home">
            <Header></Header>
            <div className="home-open" id="home-open">
                <div className="home-tip">
                    构思中......
                </div>
            </div>
        </div>
    );
}

export default Home;
