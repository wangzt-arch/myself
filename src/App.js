import React, { useState, useEffect } from "react";
import "./App.css";

const App = () => {
  const [count, setCount] = useState(0);
  useEffect(() => {
    console.log("执行了useEffect[]");
  }, []);
  useEffect(() => {
    console.log("执行了useEffect");
  });
  useEffect(() => {
    console.log("执行了useEffect count");
  }, [count]);
  const onChangeCount = () => {
    console.log(count);
    setCount(count + 1);
  };
  return (
    <div className="App">
      {count}
      <button onClick={onChangeCount}>点击</button>
    </div>
  );
};

export default App;
