import React from 'react'
import { useNavigate } from 'react-router-dom';
import Header from '../../components/Header'


function Home() {
    const navigate = useNavigate()
    return (
        <div onClick={() => navigate('/myself/docs')}>
            <Header></Header>
        </div>
    )
}

export default Home