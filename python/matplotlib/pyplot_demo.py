import matplotlib.pyplot as plt
import numpy as np
# help(plt)


# refer to https://blog.csdn.net/sinat_36219858/article/details/79800460
# fmt = '[color][marker][line]'
#单条线：
# plot([x], y, [fmt], data=None, **kwargs)
#多条线一起画
# plot([x], y, [fmt], [x2], y2, [fmt2], ..., **kwargs)

# plot(x,y2,color='green', marker='o', linestyle='dashed', linewidth=1, markersize=6)
# plot(x,y3,color='#900302',marker='+',linestyle='-')

# # color:
# =============    ===============================
#     character        color
# =============    ===============================
#     ``'b'``          blue 蓝
#     ``'g'``          green 绿
#     ``'r'``          red 红
#     ``'c'``          cyan 蓝绿
#     ``'m'``          magenta 洋红
#     ``'y'``          yellow 黄
#     ``'k'``          black 黑
#     ``'w'``          white 白
# =============    ===============================

# # marker:
# =============    ===============================
#     character        description
# =============    ===============================
#     ``'.'``          point marker
#     ``','``          pixel marker
#     ``'o'``          circle marker
#     ``'v'``          triangle_down marker
#     ``'^'``          triangle_up marker
#     ``'<'``          triangle_left marker
#     ``'>'``          triangle_right marker
#     ``'1'``          tri_down marker
#     ``'2'``          tri_up marker
#     ``'3'``          tri_left marker
#     ``'4'``          tri_right marker
#     ``'s'``          square marker
#     ``'p'``          pentagon marker
#     ``'*'``          star marker
#     ``'h'``          hexagon1 marker
#     ``'H'``          hexagon2 marker
#     ``'+'``          plus marker
#     ``'x'``          x marker
#     ``'D'``          diamond marker
#     ``'d'``          thin_diamond marker
#     ``'|'``          vline marker
#     ``'_'``          hline marker
# =============    ===============================

# # linestyle
# =============    ===============================
#     character        description
# =============    ===============================
#     ``'-'``          solid line style 实线
#     ``'--'``         dashed line style 虚线
#     ``'-.'``         dash-dot line style 点画线
#     ``':'``          dotted line style 点线
# =============    ===============================

# https://blog.csdn.net/sinat_36219858/article/details/79800460
# ========================================================================================================
a=np.random.random((9,3))*2 #随机生成y
 
y1=a[0:,0]
y2=a[0:,1]
y3=a[0:,2]
 
x=np.arange(1,10)
 
ax = plt.subplot(111)
width=10
hight=3
ax.arrow(0,0,0,hight,width=0.01,head_width=0.1, head_length=0.3,length_includes_head=True,fc='k',ec='k')
ax.arrow(0,0,width,0,width=0.01,head_width=0.1, head_length=0.3,length_includes_head=True,fc='k',ec='k')
 
ax.axes.set_xlim(-0.5,width+0.2)
ax.axes.set_ylim(-0.5,hight+0.2)
 
plotdict = { 'dx': x, 'dy': y1 }
ax.plot('dx','dy','bD-',data=plotdict)
 
ax.plot(x,y2,'r^-')
ax.plot(x,y3,color='#900302',marker='*',linestyle='-')
plt.show()
# ========================================================================================================

# ========================================================================================================
x = np.arange(0, 2*np.pi, 0.02)  
y = np.sin(x)  
y1 = np.sin(2*x)  
y2 = np.sin(3*x)  
ym1 = np.ma.masked_where(y1 > 0.5, y1)  
ym2 = np.ma.masked_where(y2 < -0.5, y2)  
  
lines = plt.plot(x, y, x, ym1, x, ym2, 'o')  
#设置线的属性
plt.setp(lines[0], linewidth=1)  
plt.setp(lines[1], linewidth=2)  
plt.setp(lines[2], linestyle='-',marker='^',markersize=4)  
#线的标签
plt.legend(('No mask', 'Masked if > 0.5', 'Masked if < -0.5'), loc='upper right')  
plt.title('Masked line demo')  
plt.show()
# ========================================================================================================

# ========================================================================================================
theta = np.arange(0, 2*np.pi, 0.01)
xx = [1,2,3,10,15,8]
yy = [1,-1,0,0,7,0]
rr = [7,7,3,6,9,9]
 
fig = plt.figure()
axes = fig.add_subplot(111)
 
i = 0
while i < len(xx):
    x = xx[i] + rr[i] *np.cos(theta)
    y = yy[i] + rr[i] *np.cos(theta)
    axes.plot(x,y)
    axes.plot(xx[i], yy[i], color='#900302', marker='*')
    i = i+1
width = 20
hight = 20
axes.arrow(0,0,0,hight,width=0.01,head_width=0.1,head_length=0.3,fc='k',ec='k')
axes.arrow(0,0,width,0,width=0.01,head_width=0.1,head_length=0.3,fc='k',ec='k')
plt.show()
# ========================================================================================================