import React from 'react'
import Header from '../../components/Header'
import { useNavigate } from 'react-router-dom';


function Docs() {
    const navigate = useNavigate()
    return (
        <div onClick={() => { navigate('/myself/home') }}>
            <Header></Header>
            Docs页面
        </div>
    )
}

export default Docs