import React from "react";
import Header from "../../components/Header";
import { useState, useRef } from "react";
import { getTranslate } from "../../api"
import "./index.scss";

function Home() {
  let [transRes, setTransRes] = useState('');
  let [language, setLanguage] = useState('en');
  let [transInp, setTransInp] = useState('');
  const textArea = useRef(null)
  const inputWords = (e) => {
    setTransInp(e.target.value.trim())
  }
  const languageChange = (e) => {
    setLanguage(e.target.value)
  }
  const translate = async () => {
    try {
      let res = await getTranslate(transInp, language)
      setTransRes(res.data.target)
    } catch (error) {
      console.log(error);
    }
  }
  return (
    <div className="home">
      <Header></Header>
      <div>
        <button onClick={translate}>翻译</button>
        <select name="" id="" onChange={languageChange}>
          <option value="en" label="英文"></option>
          <option value="zh" label="中文"></option>
        </select>
      </div>
      <div className="translate-box">
        <textarea
          ref={textArea}
          style={{ width: '50%', fontSize: '18px'}}
          rows="5" onChange={(e) => inputWords(e)} autoFocus>
        </textarea>
        <textarea
          style={{ width: '50%', fontSize: '18px' }}
          rows="5" value={transRes} readOnly >
        </textarea>
      </div>
    </div>
  );
}

export default Home;
