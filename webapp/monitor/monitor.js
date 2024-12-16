// Kết nối tới ROS
const ros = new ROSLIB.Ros({
    url: 'ws://192.168.1.100:9090' // Thay <ROS_IP> bằng địa chỉ ROSBridge của bạn
});
ros.on('connection', () => {
    console.log('Connected to ROSBridge');
});
ros.on('error', (error) => {
    console.error('Error connecting to ROSBridge:', error);
});


// // Laser scan
// const canvas = document.getElementById('laser-canvas');
// const ctx = canvas.getContext('2d');
// const width = canvas.width;
// const height = canvas.height;
// const laserTopic = new ROSLIB.Topic({
//     ros: ros,
//     name: '/scan', // Thay bằng topic LaserScan của bạn
//     messageType: 'sensor_msgs/LaserScan'
// });
// laserTopic.subscribe((message) => {
//     console.log('LaserScan received:', message);
//     const { ranges, angle_min, angle_max, angle_increment } = message;
//     // Xóa canvas
//     ctx.clearRect(0, 0, width, height);
//     // Vẽ laser scan
//     ctx.fillStyle = 'red';
//     const centerX = width / 2;
//     const centerY = height / 2;
//     const scale = 100; // Tỉ lệ (căn chỉnh để phù hợp với dữ liệu)
//     ranges.forEach((range, index) => {
//         const angle = angle_min + index * angle_increment;
//         const x = centerX + range * Math.cos(angle) * scale;
//         const y = centerY - range * Math.sin(angle) * scale;
//         // Vẽ điểm
//         ctx.beginPath();
//         ctx.arc(x, y, 2, 0, 2 * Math.PI);
//         ctx.fill();
//     });
// });



// // Map
const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');
const width = canvas.width;
const height = canvas.height;
// Hiển thị OccupancyGrid
const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/map', // Thay bằng topic OccupancyGrid của bạn
    messageType: 'nav_msgs/OccupancyGrid'
});
mapTopic.subscribe((message) => {
    console.log('OccupancyGrid received:', message);
    const { data, info } = message;
    const { width: mapWidth, height: mapHeight } = info;
    // Tính toán kích thước mỗi ô (cell)
    const cellWidth = width / mapWidth;
    const cellHeight = height / mapHeight;
    // Xóa canvas
    ctx.clearRect(0, 0, width, height);
    // Duyệt qua từng ô trong OccupancyGrid
    for (let y = 0; y < mapHeight; y++) {
        for (let x = 0; x < mapWidth; x++) {
            const index = y * mapWidth + x;
            const value = data[index];
            // Chọn màu dựa trên giá trị occupancy
            let color;
            if (value === -1) {
                color = 'gray'; // Unknown
            } else if (value === 0) {
                color = 'white'; // Free space
            } else {
                color = 'black'; // Occupied
            }
            // Vẽ ô
            ctx.fillStyle = color;
            ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
        }
    }
});


