#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        for(std::size_t x = 0; x < input.GetSizeX(); ++x)
            for(std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                float r = (float)(rand())/RAND_MAX-0.5;
                output.At(x, y) = {
                    (color.r+r)>0.5?1:0,
                    (color.g+r)>0.5?1:0,
                    (color.b+r)>0.5?1:0,
                };
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        for(std::size_t x = 0; x < input.GetSizeX(); ++x)
            for(std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                glm::vec3 n = noise.At(x%noise.GetSizeX(),y%noise.GetSizeY());
                output.At(x, y) = {
                    (color.r+n.r-0.5)>0.5?1:0,
                    (color.g+n.g-0.5)>0.5?1:0,
                    (color.b+n.b-0.5)>0.5?1:0,
                };
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        for(std::size_t x = 0; x < input.GetSizeX(); ++x)
            for(std::size_t y = 0; y < input.GetSizeY(); ++y){
                glm::vec3 color =input.At(x,y);
                output.At(3*x,3*y)={
                    color.r>0.66?1:0,
                    color.g>0.66?1:0,
                    color.b>0.66?1:0,
                };
                output.At(3*x,3*y+1)={
                    color.r>0.88?1:0,
                    color.g>0.88?1:0,
                    color.b>0.88?1:0,
                };
                output.At(3*x,3*y+2)={
                    color.r>0.44?1:0,
                    color.g>0.44?1:0,
                    color.b>0.44?1:0,
                };
                output.At(3*x+1,3*y)={
                    color.r>0.11?1:0,
                    color.g>0.11?1:0,
                    color.b>0.11?1:0,
                };
                output.At(3*x+1,3*y+1)={
                    color.r>0?1:0,
                    color.g>0?1:0,
                    color.b>0?1:0,
                };
                output.At(3*x+1,3*y+2)={
                    color.r>0.33?1:0,
                    color.g>0.33?1:0,
                    color.b>0.33?1:0,
                };
                output.At(3*x+2,3*y)={
                    color.r>0.55?1:0,
                    color.g>0.55?1:0,
                    color.b>0.55?1:0,
                };
                output.At(3*x+2,3*y+1)={
                    color.r>0.22?1:0,
                    color.g>0.22?1:0,
                    color.b>0.22?1:0,
                };
                output.At(3*x+2,3*y+2)={
                    color.r>0.77?1:0,
                    color.g>0.77?1:0,
                    color.b>0.77?1:0,
                };
            }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        ImageRGB inputcopy = input;
        for(std::size_t x = 0; x < input.GetSizeX(); ++x)
            for(std::size_t y = 0; y < input.GetSizeY(); ++y){
                glm::vec3 color = inputcopy.At(x,y);
                glm::vec3 newcolor = {
                    color.r>0.5?1:0,
                    color.g>0.5?1:0,
                    color.b>0.5?1:0,
                };
                output.At(x,y)=newcolor;
                glm::vec3 error = color - newcolor;
                glm::vec3 temp1,temp2;
                if(x+1<input.GetSizeX())
                {
                    temp1=error*glm::vec3(7.0/16);
                    temp2=inputcopy.At(x+1,y);
                    temp1=temp1+temp2;
                    inputcopy.At(x+1,y)=temp1;
                }
                if(x>0 && y+1<input.GetSizeY())//x-1>=0 is wrong
                {
                    temp1=error*glm::vec3(3.0/16);
                    temp2=inputcopy.At(x-1,y+1);
                    temp1=temp1+temp2;
                    inputcopy.At(x-1,y+1)=temp1;
                }
                if(y+1<input.GetSizeY())
                {
                    temp1=error*glm::vec3(5.0/16);
                    temp2=inputcopy.At(x,y+1);
                    temp1=temp1+temp2;
                    inputcopy.At(x,y+1)=temp1;
                }
                if(x+1<input.GetSizeX() && y+1<input.GetSizeY())
                {
                    temp1=error*glm::vec3(1.0/16);
                    temp2=inputcopy.At(x+1,y+1);
                    temp1=temp1+temp2;
                    inputcopy.At(x+1,y+1)=temp1;
                }
            }
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        for(std::size_t x = 0; x < input.GetSizeX(); ++x)
            for(std::size_t y = 0; y < input.GetSizeY(); ++y){
                glm::vec3 color=glm::vec3(0,0,0);
                int count=0;
                for(int i=-1;i<=1;i++)
                    for(int j=-1;j<=1;j++)
                    {
                        if(x+i>=0&&x+i<input.GetSizeX()&&y+j>=0&&y+j<input.GetSizeY())
                        {
                            color=color+input.At(x+i,y+j);
                            count++;
                        }
                    }
                output.At(x,y)=color/(float)count;
            }
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        for(std::size_t x = 0; x < input.GetSizeX(); ++x)
            for(std::size_t y = 0; y < input.GetSizeY(); ++y){
                glm::vec3 Gx=glm::vec3(0,0,0), Gy=glm::vec3(0,0,0);
                for(int i=-1;i<=1;i++)
                    for(int j=-1;j<=1;j++)
                    {
                        if(x+i>=0&&x+i<input.GetSizeX()&&y+j>=0&&y+j<input.GetSizeY())
                        {
                            glm::vec3 color = input.At(x+i,y+j);
                            Gx=Gx+color*glm::vec3((j==0?0:(j>0?1:-1))*(i==0?2:1));
                            Gy=Gy+color*glm::vec3((i==0?0:(i>0?-1:1))*(j==0?2:1));
                        }
                    }
                output.At(x,y)=glm::sqrt(Gx*Gx+Gy*Gy);
            }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            g[y*width]=inputBack.At(offset.x,offset.y+y)-inputFront.At(0,y);
            g[y*width+width-1]=inputBack.At(offset.x+width-1,offset.y+y)-inputFront.At(width-1,y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            g[x]=inputBack.At(offset.x+x,offset.y)-inputFront.At(x,0);
            g[(height-1)*width+x]=inputBack.At(offset.x+x,offset.y+height-1)-inputFront.At(x,height-1);

        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
            int dx=abs(p1.x-p0.x), dy=abs(p1.y-p0.y);
            int sx=p0.x<p1.x?1:-1, sy=p0.y<p1.y?1:-1;
            int err=dx-dy;
            int cx=p0.x, cy=p0.y;
            while(1)
            {
                canvas.At(cx,cy)=color;
                if(cx==p1.x&&cy==p1.y) break;
                int err2=err*2;
                if(err2>-dy)
                {
                    err-=dy;
                    cx+=sx;
                }
                else
                {
                    err+=dx;
                    cy+=sy;
                }
            }
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
            glm::ivec2 points[3]={p0,p1,p2};
            if(points[0].y>points[1].y)std::swap(points[0],points[1]);
            if(points[0].y>points[2].y)std::swap(points[0],points[2]);
            if(points[1].y>points[2].y)std::swap(points[1],points[2]);
            glm::ivec2 pminy=points[0],
                       pmidy=points[1],
                       pmaxy=points[2];
            for(int y=pminy.y;y<=pmaxy.y;y++)
            {
                if(y<pmidy.y)
                {
                    if (pmidy.y==pminy.y)continue;
                    float t1=(float)(y-pminy.y)/(pmidy.y-pminy.y);
                    int x1=pminy.x+(int)((pmidy.x-pminy.x)*t1);
                    float t2=(float)(y-pminy.y)/(pmaxy.y-pminy.y);
                    int x2=pminy.x+(int)((pmaxy.x-pminy.x)*t2);
                    if(x1>x2)std::swap(x1,x2);
                    for(int x=x1;x<=x2;x++)canvas.At(x,y)=color;
                }
                else
                {
                    if(pmaxy.y==pmidy.y){
                        int x1=pmidy.x;
                        int x2=pmaxy.x;
                        if(x1>x2)std::swap(x1,x2);
                        for(int x=x1;x<=x2;x++)canvas.At(x,y)=color;
                        continue;
                    }
                    float t1=(float)(y-pmidy.y)/(pmaxy.y-pmidy.y);
                    int x1=pmidy.x+(int)((pmaxy.x-pmidy.x)*t1);
                    float t2=(float)(y-pminy.y)/(pmaxy.y-pminy.y);
                    int x2=pminy.x+(int)((pmaxy.x-pminy.x)*t2);
                    if(x1>x2)std::swap(x1,x2);
                    for(int x=x1;x<=x2;x++)canvas.At(x,y)=color;
                }
            }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        int outW=output.GetSizeX(),
            outH=output.GetSizeY(),
            inW=input.GetSizeX(),
            inH=input.GetSizeY();
        for(int y=0;y<outH;++y){
            for(int x=0;x<outW;++x){
                glm::vec3 colorSum(0.0f);
                for(int dy=0;dy<rate;++dy){
                    for(int dx=0;dx<rate;++dx){
                        float fx=(x+(dx*1.0)/rate)*inW/outW;
                        float fy=(y+(dy*1.0)/rate)*inH/outH;
                        int ix=int(fx),iy=int(fy);
                        float tx=fx-ix,ty=fy-iy;//weight
                        int ix1=std::min(ix+1,int(inW)-1);
                        int iy1=std::min(iy+1,int(inH)-1);
                        glm::vec3 c00=input.At(ix,iy),c10=input.At(ix1,iy);
                        glm::vec3 c01=input.At(ix,iy1),c11=input.At(ix1,iy1);
                        glm::vec3 c0=c00*(1-tx)+c10*tx,c1=c01*(1-tx)+c11*tx;
                        glm::vec3 c=c0*(1-ty)+c1*ty;
                        colorSum+=c;
                    }
                }
                output.At(x,y)=colorSum/float(rate*rate);
            }
        }
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
            if(points.size()==1)return points[0];
            std::vector<glm::vec2> newpoints;
            for(int i=0;i<points.size()-1;i++)
                newpoints.push_back(points[i]*(1-t)+points[i+1]*t);
            return CalculateBezierPoint(newpoints,t);
            //return glm::vec2 {0, 0};
    }
} // namespace VCX::Labs::Drawing2D