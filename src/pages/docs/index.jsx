import React, { useEffect, useState } from 'react'
import Header from '../../components/Header'
import docs from '../../docs/regex.md'
import ReactMarkdown from 'react-markdown'

function Docs() {
    let [aaa, setAaa] = useState()
    useEffect(() => {
        fetch(docs).then(res => res.text()).then(text => {setAaa(text)})
    }, [aaa])
    return (
        <div>
            <Header></Header>
            <div className="docs">
                <ReactMarkdown children={aaa}></ReactMarkdown>
            </div>
        </div>
    )
}

export default Docs