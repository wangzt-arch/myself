import React, { useRef } from "react";
import Header from "../../components/Header";
// import Line from '../../components/Line'
import { useEffect } from "react";
import { getTranslate } from "../../api"
import party from "party-js";
import "./index.css";

function Home() {
    const partyRef = useRef();
    function onParty() {
        party.confetti(partyRef.current, {
            count: party.variation.range(20, 40)
        });
    }
    useEffect(() => {
        console.log('s');
        // getTranslate('word')
    }, [])
    return (
        <div className="home">
            <Header></Header>
            <div className="home-open" id="home-open"
                ref={partyRef}
                onClick={onParty}>
                {/* <Line></Line> */}
                {/* <div className="home-content"></div> */}
                <div className="home-tip">
                    构思中......点击试试吧
                </div>
            </div>
        </div>
    );
}

export default Home;
