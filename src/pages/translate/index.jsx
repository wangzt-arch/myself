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
  const clear = () => {
    textArea.current.value = ''
    setTransInp('')
    textArea.current.focus()
  }
  const copy = () => {
    const el = document.createElement('input')
    el.setAttribute('value', transRes)
    document.body.appendChild(el)
    el.select()
    document.execCommand('copy')
    document.body.removeChild(el)
  }
  const onKeyDownEnter = (e) => {
    if (e.code == "Enter") {
      translate()
    }
  }
  return (
    <div className="home">
      <Header></Header>
      <div>
        <button onClick={translate} style={{ marginRight: '10px' }}>翻译</button>
        <label>自动检测——{">"} </label>
        <select name="" id="" onChange={languageChange}>
          <option value="en" label="英文"></option>
          <option value="zh" label="中文"></option>
        </select>
      </div>
      <div className="translate-box">
        {transInp && <div className="textarea-clear" onClick={clear}>X</div>}
        <div className="textarea-copy" onClick={copy}>三</div>
        <textarea
          onKeyDown={onKeyDownEnter}
          className="textarea-input"
          ref={textArea}
          style={{ width: '50%', fontSize: '18px', padding: '10px', resize: 'none' }}
          rows="5" onChange={inputWords} autoFocus>
        </textarea>
        <textarea
          className="textarea-show"
          style={{ width: '50%', fontSize: '18px', resize: 'none' }}
          rows="5" value={transRes} readOnly >
        </textarea>
      </div>
    </div>
  );
}

export default Home;
