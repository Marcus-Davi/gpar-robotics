accx = ones(100,1);

tam = length(accx);

velocity = zeros(tam,1);
position = zeros(tam,1);
dt = 1/100;

for i=2:tam
    velocity(i) = velocity(i-1) + ((accx(i)+accx(i-1)))/2*dt;
    position(i) = position(i-1) + ((velocity(i)+velocity(i-1))/2)*dt;
end