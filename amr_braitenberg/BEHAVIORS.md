# Braitenberg vehicles

## Behaviour When factor 1 and factor 2 is equal

Factor 1 value has been used for left wheel and factor 2 for right. The values have been multiplied with the sensor input. Hence the vehicle will move based on the sensor input only when the factor values are equal.

- ### Type A
    **Inverse of the normalized value of sensor input** has been applied to wheels when type is set to A. So when there will be an obstacle closer to a sensor then the corresponding wheel would move faster and the vehicle would rotate in the other direction.

    For example, if there is an obstacle closer to left sensor and no obstacle at right then the left wheel speed will be more than the right and the vehicle will rotate to right.

- ### Type B
    The vehicle behaves the opposite in case of Type B compared to type A. When configured as type B, the wheel speed is proportional to the corresponding sensor value. Hence, When there is an obstacle within range of a sensor, the corresponding wheel speed reduces compared to the other and the vehicle moves towards the obstacle and crashes.

- ### Type C
    In case of type C, both the sensors are connected to both wheels. So, both wheels would have same speed irrespective of presense of obstacle ahead.


## Behavior when factor 1 and factor 2 is not equal
Speed of the wheels is dependent not only on the sensor input but also the factors. factor 1 and factor 2 is multiplied with the sensor input values for left and right wheel respectively. Therefore, the final value of the speed depends on the factors and the vehicle may rotate left or right even when the sensor inputs are equal.

For example, In case of type C, the vehicle will rotate right if factor 2 is less thant factor 1.
