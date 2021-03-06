# drone
# Strategy
 2021 미니드론경진대회 전략으로 1단계에서는 빠른 시간내에 통과를 목적으로 하며, 2단계와 3단계에서는 정확성을 중시하며 통과를 최우선 순위로 하고, 빠른 시간은 두번째 우선순위로 삼는다. 또한 우리팀은 정확성과 속도의 향상을 위해서 image processing toolbox에서 제공되는 여러 함수들을 활용하여서 이미지 처리를 수행하였다. (rgb2hsv, imcrop, regionprops함수)  또한 장애물까지의 거리를 측정할 때 거리에 따른 1,2,3단계별로 링의 직경을 여러번 측정하여 드론이 현재 위치에서 링을 통과하기 위한 거리(Input)를 계산하여 정확성을 높였다. 코드 작성 후에 실제로 장애물을 설치하여 코드가 제대로 동작하는지 확인하였으며, 여러번 시행착오를 통해 알고리즘을 개선하였다.
# Algorithm
> 1. 드론 객체 선언 및 takeoff

> 2. 링의 구멍을 이미지 처리하여 구멍을 찾아낸다. 구멍에서 많이 벗어난 경우 상하 좌우로 이동한다.

> 3. 이미지 속에서 링의 중점을 구하고 상하좌우로 조금씩 이동하여 드론을 중점에 맞춘다.

> 4. 링의 구멍의 장축을 측정하여 드론과의 거리를 계산한 뒤 링을 통과킨다. 
> (드론과 장애물까지의 거리에 따라 링의 장축의 길이가 달라지므로 이를 이용하여 거리를 측정하였다.)

> 5. 표식을 인식하여 빨간색이면 회전, 보라색이면 착지를 한다

> 6. 착지하기 전까지 **2**단계로 돌아가 반복한다


# Source Code
Source Code를 알고리즘 단계별로 기술하였다. 상세한 설명은 주석을 통하여 확인할 수 있다.

1. 변수 설정 및 takeoff
```matlab
%초기화
clear,clc ;

%변수 설정
drone=ryze();   
cam=camera(drone);
originCenter=[480 170; 480 170; 480 170]; 
count=0;
max=0;
none=0;
takeoff(drone);
```
2. 구멍 찾기

```matlab
%level별로 반복문 설정
for level = 1:3
    %% finding hole
    
    %1단계에서는 시작시에 좌우에 대한 중점이 맞춰져 있음
    %1단계에서는 중점을 찾을 필요가 없으므로 높이에 대한 중점만 맞춰줌
    if level == 1
        moveup(drone, 'distance', 0.25)
    end
    
    while 1
        
        %1단계의 경우 구멍을 찾지 않음
        if level == 1
            break;
        end
        
        %파란색에 대한 HSV값 설정 및 이진화
        frame=snapshot(cam);
        hsv = rgb2hsv(frame);
        h = hsv(:,:,1);
        s = hsv(:,:,2);
        v = hsv(:,:,3);
        blue=(0.55<h)&(h<0.7)&(0.5<s)&(s<0.9);
        
        % 첫 행을 1로 변환
        blue(1,:) = 1;
        
        % 마지막 행을 1로 변환
        blue(720,:) = 1;
        
        %구멍을 채움
        bw2 = imfill(blue,'holes');
        
        %구멍을 채우기 전과 후를 비교하여 값이 일정하면 0, 변했으면 1로 변환
        for x=1:720
            for y=1:size(blue,2)
                if blue(x,y)==bw2(x,y)
                    bw2(x,y)=0;
                end
            end
        end
        
        %변환한 이미지의 픽셀 수가 1000이상이면 구멍을 인식했다고 파악
        %10000이하이면 상승하여 전 과정을 다시 반복
        if sum(bw2,'all')>10000
            disp('find hole!');
            break;
            
        else
            %화면의 좌우를 비교
            diff_lr = sum(imcrop(blue,[0 0 480 720]),'all') - sum(imcrop(blue,[480 0 480 720]),'all');
            diff_ud = sum(imcrop(blue,[0 0 960 360]),'all') - sum(imcrop(blue,[0 360 960 360]),'all');
            
            %장애물에 대한 이미지의 좌우 차이값이 30000이상이면 좌우로 이동
            if diff_lr > 30000
                moveleft(drone,'distance',0.5,'speed',1);
                disp('finding hole_move_left 0.5m');
                
            elseif diff_lr < -30000
                moveright(drone,'distance',0.4,'speed',1);
                disp('finding hole_move_right 0.4m');
            end
            
            %장애물에 대한 이미지의 상하 차이값이 10000이상이면 상하로 이동
            if diff_ud > 10000
                moveup(drone,'distance',0.3,'speed',1);
                disp('finding hole_move_up_0.3m');
            elseif diff_ud < -10000
                movedown(drone,'distance',0.2,'speed',1);
                disp('finding hole_move_down_0.2m');
            end
            
            %장애물이 카메라에 잡히지 않을 경우의 예외처리
            %상하좌우로 이동하며 장애물 인식
            if sum(blue,'all') < 3000
                if none==0
                    if diff_ud >= 0
                        moveup(drone,'distance',0.4,'speed',1);
                        disp('Cannot find barrier_moving_up');
                    else
                        movedown(drone,'distance',0.2,'speed',1);
                        disp('Cannot find barrier_moving_down');
                    end
                    
                    %처음에는 오른쪽으로 1m 이동
                    moveright(drone,'distance',1,'speed',1);
                    disp('Cannot find barrier_moving_right_1m');
                    none = none+1;
                    
                    %오른쪽에 장애물 없을 시 처음 위치기준 왼쪽으로 1.1m 이동
                elseif none==1
                    moveleft(drone,'distance',2.1,'speed',1);
                    disp('Cannot find barrier_moving_left_2.1m');
                    none = 2;
                end
            end
        end
    end
```

3. 장애물의 중점 찾기
```matlab
 %% find center point
    
    %같은 동작을 반복하는 loop에 빠질 경우 예외처리를 위한 변수 설정
    count_l = 0; count_r = 0; count_u = 0; count_d = 0; %*
    
    while 1
        
        %1단계의 경우 장애물의 중점을 찾지 않음
        if level == 1
            break;
        end
        
        %파란색에 대한 HSV값 설정 및 이진화
        frame=snapshot(cam);
        hsv = rgb2hsv(frame);
        h = hsv(:,:,1);
        s = hsv(:,:,2);
        v = hsv(:,:,3);
        blue=(0.55<h)&(h<0.7)&(0.5<s)&(s<0.9);
        
        %첫 행을 1로 변환
        blue(1,:)=1;
        
        %마지막 행을 1로 변환
        blue(720,:)=1;
        
        %구멍을 채움
        bw2 = imfill(blue,'holes');
        
        %구멍을 채우기 전과 후를 비교하여 값이 일정하면 0, 변했으면 1로 변환
        for x=1:720
            for y=1:size(blue,2)
                if blue(x,y)==bw2(x,y)
                    bw2(x,y)=0;
                end
            end
        end
        
        %이미지에서 인식된 곳들의 중점과 보조축의 크기를 구함
        stats = regionprops('table',bw2, 'Centroid', 'MinorAxisLength');
        z = stats.MinorAxisLength;
        max_r=0;
        y=stats.Centroid;
        
        %보조축의 크기가 가장 큰 곳의 중점을 가져옴
        for i=1:size(stats)
            if z(i,1)>=max_r
                max_r=z(i,1);
                firstCenter(1,1)=round(y(i,1));
                firstCenter(1,2)=round(y(i,2));
            end
        end
        clearvars max
        
        %측정된 중점과 이상 중점을 비교하여 이동
        ct_diff_lr = firstCenter(1,1)-originCenter(1,1);
        ct_diff_ud = firstCenter(1,2)-originCenter(1,2);
        
        % x축에 대한 중점 찾기
        if ct_diff_lr >= 40 && ct_diff_lr <= 80
            moveright(drone,'Distance',0.2,'speed',1);
            disp('finding center point_move right_0.2m');
            count_r = count_r + 1; %*
            
        elseif ct_diff_lr > 80
            moveright(drone,'Distance',0.3,'speed',1);
            disp('finding center point_move right_0.3m');
            count_r = count_r + 1; %*
            
        elseif ct_diff_lr <= -40 && ct_diff_lr >= -80
            moveleft(drone,'Distance',0.25,'speed',1);
            disp('finding center point_move left_0.25m');
            count_l = count_l + 1; %*
            
        elseif ct_diff_lr < -80
            moveleft(drone,'Distance',0.3,'speed',1);
            disp('finding center point_move left_0.3m');
            count_l = count_l + 1; %*
            
        end
        
        % y축에 대한 중점찾기
        if ct_diff_ud >= 30 && ct_diff_ud <= 75
            movedown(drone,'Distance',0.2,'speed',1);
            disp('finding center point_move down_0.2m');
            count_d = count_d + 1; %*
            
        elseif ct_diff_ud > 75
            movedown(drone,'Distance',0.3,'speed',1);
            disp('finding center point_move down_0.3m');
            count_d = count_d + 1; %*
            
        elseif ct_diff_ud <= -30 && ct_diff_ud >= -75
            moveup(drone,'Distance',0.25,'speed',1);
            disp('finding center point_move up_0.25m');
            count_u = count_u + 1; %*
            
        elseif ct_diff_ud < -75
            moveup(drone,'Distance',0.3,'speed',1);
            disp('finding center point_move up_0.3m');
            count_u = count_u + 1; %*
        end
        
        %max 함수를 사용하기 위해 벡터 정의
        tmp = [count_l, count_r, count_u, count_d]; %*
        
        %같은 동작을 반복하는 loop에 빠질 경우 예외처리
        %장애물이 가까워서 계속 같은 행동을 반복하는 loop에 갇힐 시 빠져나오기 위해 0.5m 뒤로 이동
        if max(tmp) >= 6 %*
            disp('a drone is in loop. moveback_0.3m'); %*
            moveback(drone, 'distance', 0.5); %*
            count_l=0; count_r=0; count_u=0; count_d=0; %*
        end %*
        
        %오차범위 내에 있으면 반복문 탈출(드론이 장애물 중점 위치로 이동 완료)
        if ct_diff_ud < 30 && ct_diff_ud > -30 && ct_diff_lr < 40 && ct_diff_lr > -40
            disp('find center point!');
            count_l=0; count_r=0; count_u=0; count_d=0; %*
            break;
        end
    end
```
4. 장애물까지의 거리 측정 및 이동
```matlab
 %% measure distance from barrier and moving
    
    %파란색에 대한 HSV값 설정 및 이진화
    frame = snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    v = hsv(:,:,3);
    blue = (0.55<h)&(h<0.7)&(0.5<s)&(s<0.9);
    
    % 첫 행을 1로 변환
    blue(1,:) = 1;
    
    % 마지막 행을 1로 변환
    blue(720,:) = 1;
    
    %구멍을 채움
    bw2 = imfill(blue,'holes');
    
    %구멍을 채우기 전과 후를 비교하여 값이 일정하면 0, 변했으면 1로 변환
    for x=1:720
        for y=1:size(blue,2)
            if blue(x,y)==bw2(x,y)
                bw2(x,y)=0;
            end
        end
    end
    
    %이미지에서 인식된 곳들의 장축의 크기를 구함
    stats = regionprops('table', bw2, 'MajorAxisLength');
    long_rad = max(stats.MajorAxisLength);
    
    %장애물로 부터 드론까지의 거리를 구하기 위한 알고리즘
    %장매물까지의 거리와 장축의 길이를 1대1로 대응하여 위치 추정
    
    %md의 첫번째 행은 1단계(지름 78cm) 1m부터 3m까지 0.3m간격으로 측정시 장축의 길이
    %md의 두번째 행은 2단계(지름 57cm) 1m부터 3m까지 0.3m간격으로 측정시 장축의 길이
    %md의 세번째 행은 3단계(지름 50cm) 1m부터 3m까지 0.3m간격으로 측정시 장축의 길이
    md = [860 670 530 440 370 315 285; 550 450 370 320 265 230 200; 460 380 310 255 225 198 175];
    
    %각 단게에서 long_rad의 값에 따라서 거리 추정
    if level == 1 && sum(bw2,'all') <= 10000
        moveforward(drone, 'distance', 1.4, 'speed', 1);
        disp('측정 거리 = 1m');
        disp('이동거리 = 1.4m');
        long_rad
        
    elseif long_rad > md(level,1)
        moveforward(drone, 'distance', 1.4, 'speed', 1);
        disp('측정 거리 = 1m');
        disp('이동거리 = 1.4m');
        long_rad
        
    elseif long_rad > md(level,2) && long_rad <= md(level,1)
        moveforward(drone, 'distance', 1.6, 'speed', 1);
        disp('측정 거리 1m~1.2m');
        disp('이동거리 = 1.6m');
        long_rad
        
    elseif long_rad > md(level,3) && long_rad <= md(level,2)
        moveforward(drone, 'distance', 1.9, 'speed', 1);
        disp('측정 거리 1.2m~1.5m');
        disp('이동거리 = 1.9m');
        long_rad
        
    elseif long_rad > md(level,4) && long_rad <= md(level,3)
        moveforward(drone, 'distance', 2.2, 'speed', 1);
        disp('측정 거리 1.5m~1.8m');
        disp('이동거리 = 2.2m');
        long_rad
        
    elseif long_rad > md(level,5) && long_rad <= md(level,4)
        moveforward(drone, 'distance', 2.5, 'speed', 1);
        disp('측정 거리 1.8m~2.1m');
        disp('이동거리 = 2.5m');
        long_rad
        
    elseif long_rad > md(level,6) && long_rad <= md(level,5)
        moveforward(drone, 'distance', 2.8, 'speed', 1);
        disp('측정 거리 2.1m~2.4m');
        disp('이동거리 = 2.8m');
        long_rad
        
    elseif long_rad > md(level,7) && long_rad <= md(level,6)
        moveforward(drone, 'distance', 3.1, 'speed', 1);
        disp('측정 거리 2.4m~2.7m');
        disp('이동거리 = 3.1m');
        long_rad
        
    elseif long_rad <= md(level,7)
        moveforward(drone, 'distance', 3.4, 'speed', 1);
        disp('측정 거리 2.7m~3m');
        disp('이동거리 = 3.4m');
        long_rad
        
    end
```
5. 빨간색 점 및 보라색 점 찾은 후 회전 혹은 착지
```matlab
 %% detecting sign
    
    % 1,2 단계일 때 수행
    if level==1 || level==2
        %빨간점 찾기
        while 1
            
            %빨간색에 대한 HSV값 설정 및 이진화
            frame=snapshot(cam);
            hsv = rgb2hsv(frame);
            h = hsv(:,:,1);
            s = hsv(:,:,2);
            v = hsv(:,:,3);
            red = ((0.95<h) & (h<1) | (0<h) & (h<0.05)) & (0.8<s) & (s<=1);
            
            %빨간색의 픽셀이 400이 넘으면 90도 회전
            if sum(red,'all')>400
                if count==1
                    moveforward(drone,'distance',0.2);
                    count=0;
                end
                
                turn(drone,deg2rad(-90))
                break;
                
            else
                moveback(drone,'distance',0.2)
                count=1;
            end
        end
        
        moveforward(drone,'Distance',1.1 ,'speed',1);
        
        %3단계일 때 실행
    elseif level==3
        %파란점 찾기
        while 1
            %파란색에 대한 HSV값 설정 및 이진화
            frame=snapshot(cam);
            hsv = rgb2hsv(frame);
            h = hsv(:,:,1);
            s = hsv(:,:,2);
            v = hsv(:,:,3);
            blue = (0.55<h) & (h<0.8) & (0.6<s) & (s<=1);
            
            %파란색의 픽셀이 300이 넘으면 착지
            if sum(blue,'all')>300
                if count==1
                    moveforward(drone,'distance',0.2);
                    count=0;
                end
                land(drone);
                break;
            else
                moveback(drone,'distance',0.2);
                count=1;
            end
        end
    end
end

clear drone;
clear cam;
```
