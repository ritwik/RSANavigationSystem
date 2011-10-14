b = []

from matplotlib.pyplot import figure, show
from matplotlib.patches import Ellipse
import numpy as np

if 1:
    fig = figure(1,figsize=(8,5))
    fig.clf()
    
   # bx = fig.add_subplot(111, autoscale_on=False, xlim=(-3,3), ylim=(-2,2))
   # bx.plot([3, 3, -3, -3],[-3,3,-3,3])

    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-4,4), ylim=(-3,3))
    ax.plot([3, 3, -3, -3,3],[-2,2,2,-2,-2], 'r')

    els = [[-1,2],[3,0],[-1,-2]]
    r = 0.25
    for el in els:
      e = Ellipse((el[0], el[1]), r, r)
      e.set_color('g')
      r *= 1.5
      ax.add_patch(e)
    # TODO implement the radius properly
    
    bx = fig.add_subplot(111, autoscale_on=False, xlim=(-4,4), ylim=(-3,3))
    bx.scatter([.1,.1],[.2,.4])
    bx.scatter([.2,.2],[.2,.4])
    
    """
    ax.annotate('$->$', xy=(2., -1),  xycoords='data',
                xytext=(-150, -140), textcoords='offset points',
                bbox=dict(boxstyle="round", fc="0.8"),
                arrowprops=dict(arrowstyle="->",
                                patchB=el,
                                connectionstyle="angle,angleA=90,angleB=0,rad=10"),
                )

    ax.annotate('fancy', xy=(2., -1),  xycoords='data',
                xytext=(-100, 60), textcoords='offset points',
                size=20,
                #bbox=dict(boxstyle="round", fc="0.8"),
                arrowprops=dict(arrowstyle="fancy",
                                fc="0.6", ec="none",
                                patchB=el,
                                connectionstyle="angle3,angleA=0,angleB=-90"),
                )

    ax.annotate('simple', xy=(2., -1),  xycoords='data',
                xytext=(100, 60), textcoords='offset points',
                size=20,
                #bbox=dict(boxstyle="round", fc="0.8"),
                arrowprops=dict(arrowstyle="simple",
                                fc="0.6", ec="none",
                                patchB=el,
                                connectionstyle="arc3,rad=0.3"),
                )

    ax.annotate('wedge', xy=(2., -1),  xycoords='data',
                xytext=(-100, -100), textcoords='offset points',
                size=20,
                #bbox=dict(boxstyle="round", fc="0.8"),
                arrowprops=dict(arrowstyle="wedge,tail_width=0.7",
                                fc="0.6", ec="none",
                                patchB=el,
                                connectionstyle="arc3,rad=-0.3"),
                )


    ann = ax.annotate('wedge', xy=(2., -1),  xycoords='data',
                xytext=(0, -45), textcoords='offset points',
                size=20,
                bbox=dict(boxstyle="round", fc=(1.0, 0.7, 0.7), ec=(1., .5, .5)),
                arrowprops=dict(arrowstyle="wedge,tail_width=1.",
                                fc=(1.0, 0.7, 0.7), ec=(1., .5, .5),
                                patchA=None,
                                patchB=el,
                                relpos=(0.2, 0.8),
                                connectionstyle="arc3,rad=-0.1"),
                )

    ann = ax.annotate('wedge', xy=(2., -1),  xycoords='data',
                xytext=(35, 0), textcoords='offset points',
                size=20, va="center",
                bbox=dict(boxstyle="round", fc=(1.0, 0.7, 0.7), ec="none"),
                arrowprops=dict(arrowstyle="wedge,tail_width=1.",
                                fc=(1.0, 0.7, 0.7), ec="none",
                                patchA=None,
                                patchB=el,
                                relpos=(0.2, 0.5),
                                )
                )
    """
  
show()
