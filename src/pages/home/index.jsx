import React, { useRef } from "react";
import Header from "../../components/Header";
// import Line from '../../components/Line'
import party from "party-js";
import "./index.css";

function Home() {
  const partyRef = useRef();
  function onParty() {
    party.confetti(partyRef.current, {
      count: party.variation.range(20, 40),
    });
  }
  return (
    <div className="home">
      <Header></Header>
      <div
        className="home-open"
        id="home-open"
        ref={partyRef}
        onClick={onParty}
      >
        {/* <Line></Line> */}
        {/* <div className="home-content"></div> */}
        还没想好写什么的部分,点击试试吧
      </div>
    </div>
  );
}

export default Home;
