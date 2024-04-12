import  React from "react";
import Header from "../../components/Header";
import { useEffect, useRef } from "react";
import "./index.css";

function Preview() {



    const sceneRef = useRef();
    useEffect(() => {
        console.log('s');

    }, [])
    return (
        <div className="model-preview">
            <Header></Header>

            <div className="model-preview__area" ref={sceneRef}>
            {/* <Canvas></Canvas> */}
            </div>
            <div className="choose-model">加载模型</div>
        </div>
    );
}

export default Preview;
