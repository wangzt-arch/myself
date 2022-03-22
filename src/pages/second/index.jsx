import { useRouter } from 'next/router'


const Second = () => {
    const router = useRouter()
    const handleClick = () => {
        router.push('/')
    }

    return <div onClick={handleClick}>second</div>
}
export default Second