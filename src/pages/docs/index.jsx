import React from 'react'
import ReactMarkdown from 'react-markdown'
import Header from '../../components/Header'
import docs from '../../docs/regex.md'

function Docs() {
    return (
        <div>
            <Header></Header>
            <div className="docs">
                <ReactMarkdown children={docs}></ReactMarkdown>
            </div>
        </div>
    )
}

export default Docs