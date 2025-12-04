#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    struct CoordinateStates {
        // your code here
        std::vector<Car> data;
    };

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example: 
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        static CoordinateStates states(data);
        SetBackGround(input, glm::vec4(1));
        float x[7]={0.05,0.2,0.35,0.5,0.65,0.8,0.95};
        for(int i=0;i<7;++i){
            DrawLine(input,glm::vec4(0,0,0,1),glm::vec2{x[i],0.1},glm::vec2{x[i],0.9},3.0f);
            DrawLine(input,glm::vec4(0.5),glm::vec2{x[i],0.1},glm::vec2{x[i],0.9},20.0f);
            DrawLine(input,glm::vec4(0.5),glm::vec2{x[i]+0.016,0.1},glm::vec2{x[i]+0.016,0.9},20.0f);

        }
        std::string data_types[7]={"cylinders","displacement","weight","horsepower","acceleration(0-60mph)","mileage","year"};
        std::string data_num_upper[7]={"9","494","5493","249","27","51","84"};
        std::string data_num_lower[7]={"2","29","1260","27","6","5","68"};
        for(int i=0;i<7;++i){
            PrintText(input,glm::vec4{0,0,0,1},glm::vec2{x[i],0.04},0.02f,data_types[i]);
            PrintText(input,glm::vec4{0,0,0,1},glm::vec2{x[i]-0.007f,0.07},0.02f,data_num_upper[i]);
            PrintText(input,glm::vec4{0,0,0,1},glm::vec2{x[i]-0.007f,0.93},0.02f,data_num_lower[i]);
        }
        int i=0;
        for(auto car:data){
            ++i;
            float color=(i*1.0f/data.size());
            glm::vec4 line_color=glm::vec4{color,0.5f,0.5f,0.5f}; // change color gradually
            float pos_y[7]={
                0.1f+0.8f*(9-car.cylinders)/(9-2),
                0.1f+0.8f*(494-car.displacement)/(494-29),
                0.1f+0.8f*(5493-car.weight)/(5493-1260),
                0.1f+0.8f*(249-car.horsepower)/(249-27),
                0.1f+0.8f*(27-car.acceleration)/(27-6),
                0.1f+0.8f*(51-car.mileage)/(51-5),
                0.1f+0.8f*(84-car.year)/(84-68)
            };
            
            if(!proxy.IsDragging())
                for(int j=0;j<6;++j)
                    DrawLine(input,line_color,glm::vec2{x[j],pos_y[j]},glm::vec2{x[j+1],pos_y[j+1]},1.0f);
            else{
                glm::vec2 start_pos=proxy.DraggingStartPoint();
                glm::vec2 end_pos=proxy.MousePos();
                float start_x=start_pos.x;
                float y_lower=std::min(start_pos.y,end_pos.y);
                float y_upper=std::max(start_pos.y,end_pos.y);
                bool selected_an_axis=false;
                for(int j=0;j<6;++j){
                    if(start_x>=x[j]-0.016f&&start_x<=x[j]+0.016f){
                        selected_an_axis=true;
                        if(pos_y[j]>=y_lower && pos_y[j]<=y_upper)
                            for(int k=0;k<6;++k)
                                DrawLine(input,line_color,glm::vec2{x[k],pos_y[k]},glm::vec2{x[k+1],pos_y[k+1]},1.0f);
                        else
                            for(int k=0;k<6;++k)
                                DrawLine(input,glm::vec4{0.5f,0.5f,0.5f,0.1f},glm::vec2{x[k],pos_y[k]},glm::vec2{x[k+1],pos_y[k+1]},1.0f);
                    }
                }
                if(!selected_an_axis)
                    for(int j=0;j<6;++j)
                        DrawLine(input,line_color,glm::vec2{x[j],pos_y[j]},glm::vec2{x[j+1],pos_y[j+1]},1.0f);
            }
            /*for(int j=0;j<6;++j)
                DrawLine(input,line_color,glm::vec2{x[j],pos_y[j]},glm::vec2{x[j+1],pos_y[j+1]},1.0f);*/
        }
        return true;
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
        int m=output.GetSizeY(), n=output.GetSizeX();
        float t=0.0f;
        for(int i=0;i<m;++i)
            for(int j=0;j<n;++j){
                float y=i,x=j;
                glm::vec3 forward_sum(0.0f), forward_total(0.0f);
                for(int k=0;k<step;++k){
                    glm::vec2 vec=field.At((int)y,(int)x);
                    float dx=vec.x, dy=vec.y;
                    float dt_x=1e9f, dt_y=1e9f;
                    if(dy>0)
                        dt_y=(std::floor(y)+1-y)/dy;
                    else if(dy<0)
                        dt_y=(std::ceil(y)-1-y)/dy;
                    if(dx>0)
                        dt_x=(std::floor(x)+1-x)/dx;
                    else if(dx<0)
                        dt_x=(std::ceil(x)-1-x)/dx;
                    float dt;
                    if(dx==0&&dy==0)dt=0;
                    else dt=std::min(dt_x,dt_y);
                    x = std::min(std::max(x + dx * dt, 0.0f), (float)(n - 1));
                    y = std::min(std::max(y + dy * dt, 0.0f), (float)(m - 1));
                    float weight = std::pow(std::cos(t+0.46*k),2);
                    forward_sum += noise.At((int)y, (int)x) * weight;
                    forward_total += glm::vec3(weight);
                }
                y=i,x=j;
                glm::vec3 backward_sum(0.0f), backward_total(0.0f);
                for(int k=0;k<step;++k){
                    glm::vec2 vec=field.At((int)y,(int)x);
                    float dx=-vec.x, dy=-vec.y;
                    float dt_x=1e9f, dt_y=1e9f;
                    if(dy>0)
                        dt_y=(std::floor(y)+1-y)/dy;
                    else if(dy<0)
                        dt_y=(std::ceil(y)-1-y)/dy;
                    if(dx>0)
                        dt_x=(std::floor(x)+1-x)/dx;
                    else if(dx<0)
                        dt_x=(std::ceil(x)-1-x)/dx;
                    float dt;
                    if(dx==0&&dy==0)dt=0;
                    else dt=std::min(dt_x,dt_y);
                    x = std::min(std::max(x + dx * dt, 0.0f), (float)(n - 1));
                    y = std::min(std::max(y + dy * dt, 0.0f), (float)(m - 1));
                    float weight = std::pow(std::cos(t-0.46*k),2);
                    backward_sum += noise.At((int)y, (int)x) * weight;
                    backward_total += glm::vec3(weight);
                }
                output.At(i,j) = (forward_sum + backward_sum) / (forward_total + backward_total);
        }
    }
}; // namespace VCX::Labs::Visualization