#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import time
from matplotlib2tikz import save as tikz_save

path = '../../../latex/images/'

pgf_with_rc_fonts = {
    "font.family": [],
    "font.serif": [],
    "font.sans-serif": [], 
    "pgf.rcfonts" : False,
}
##mpl.rcParams.update(pgf_with_rc_fonts)

def drop(df):
    return df[['num_byte', 'num_proc', 'time']].groupby(['num_proc', 'num_byte'])

def remove_proc(df):
    proc = [1,2,4,6,8,12,16]
    return df[df['num_proc'].isin(proc)]

def remove_all_proc(df):
    proc = [1]
    return df[df['num_proc'].isin(proc)]

def show_normal(df, save):
    plt.cla()
    new = df.query('type == 0')
    new = drop(new)
    new_med = new.median()
    new_stdv = new.std(ddof=0)
    new_mean = new.mean()
    print("normal, for 33554432 byte, accumulated bandwidth:")
    for i in new_med.index.get_level_values(0).unique().tolist():
        #!!!
        plt.plot(new_med.xs(i,level=0).index.values*2, new_med.xs(i, level=0), label=i)
        print(i, "%.2f" % (new_med.xs(i, level=0).values[-1][0]*i))

    #!!!
    #new_med.index.set_levels(new_med.index.levels[1]*2, level=1, inplace=True, verify_integrity=True)
    #ax = new_med.unstack(0).plot(logy=True, logx=True)
    ax = plt.gca()
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Bytes transferred per process")
    ax.set_ylabel("Bandwidth per process [MB/s]")
    ax.set_ylim(bottom=98, top=4000)
    ax.set_title(r"DMA vector copy with SVM\\for 1 - 16 processes")
    ax.grid(True, which='both', axis='y')
    ax.legend(ncol=3, loc='best', labels=new_med.index.levels[0], title="Number of processes")
    
    if save:
        tikz_save(
        path+'dma_svm.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        #extra_axis_parameters={'ytick distance=10', 'yminorgrids', 'extra y ticks=50', 'title style={align=center}'}
        extra_axis_parameters={'title style={align=center}'}
        )
    else:
        plt.show()
    return new_med, new_stdv, new_mean

def show_odp(df, save):
    plt.cla()
    new = df.query('type == 1')
    new = drop(new)
    new_med = new.median()
    new_stdv = new.std(ddof=0)
    new_mean = new.mean()
    
    print("odp, for 33554432 byte, accumulated bandwidth:")
    for i in new_med.index.get_level_values(0).unique().tolist():
        #!!!
        plt.plot(new_med.xs(i,level=0).index.values*2, new_med.xs(i, level=0), label=i)
        print("%.2f" % (new_med.xs(i, level=0).values[-1][0]*i))
    
    #!!!
    #new_med.index.set_levels(new_med.index.levels[1]*2, level=1, inplace=True, verify_integrity=True)
    #ax = new_med.unstack(0).plot(logy=True, logx=True)
    ax = plt.gca()
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Bytes transferred per process")
    ax.set_ylabel("Bandwidth per process [MB/s]")
    ax.set_title(r"DMA vector copy with SVM+ODP\\for 1 - 16 processes")
    ax.grid(True, which='both', axis='y')
    ax.legend(ncol=3, loc='best', labels=new_med.index.levels[0], title="Number of processes")
    if save:
        tikz_save(
        path+'dma_odp.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        extra_axis_parameters={'title style={align=center}'}
        #extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
        )
    else:
        plt.show()
    return new_med, new_stdv, new_mean

def show_phys(df, save):
    plt.cla()
    new = df.query('type == 2')
    new = drop(new)
    new_med = new.median()
    new_stdv = new.std(ddof=0)
    new_mean = new.mean()
    
    print("phys, for 33554432 byte, accumulated bandwidth:")
    for i in new_med.index.get_level_values(0).unique().tolist():
        #!!!
        plt.plot(new_med.xs(i,level=0).index.values*2, new_med.xs(i, level=0), label=i)
        print("%.2f" % (new_med.xs(i, level=0).values[-1][0]*i))
    
    #!!!
    #new_med.index.set_levels(new_med.index.levels[1]*2, level=1, inplace=True, verify_integrity=True)
    #ax = new_med.unstack(0).plot(logy=True, logx=True)
    ax = plt.gca()
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_ylim(bottom=98, top=4000)
    ax.set_xlabel("Bytes transferred per process")
    ax.set_ylabel("Bandwidth per process [MB/s]")
    ax.set_title(r"DMA vector copy without SVM\\for 1 - 16 processes")
    ax.grid(True, which='both', axis='y')
    ax.legend(ncol=3, loc='best', labels=new_med.index.levels[0], title="Number of processes")

    if save:
        tikz_save(
        path+'dma_phys.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        extra_axis_parameters={'title style={align=center}'}
        #extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
        )
    else:
        plt.show()
    return new_med, new_stdv, new_mean


def show_rept(df, save):
    plt.cla()
    new = df.query('type == 4')
    new = new[['num_proc', 'num_runs', 'time']].groupby(['num_proc', 'num_runs'])
    print(new.mean().index.levels[0])
    new_mean = new.mean()
    #!!!
    new_mean.index.set_levels(new_mean.index.levels[1]*2, level=1, inplace=True, verify_integrity=True)
    ax = new_mean.unstack(0).plot(logy=True, logx=False)
    ax.set_xlabel("Number of times same Vectors were reused")
    ax.set_ylabel("Bandwidth per process [MB/s]")
    ax.set_title(r"DMA vector copy with SVM\\Repeated use of same Vectors\\for 1 - 16 porcesses")
    ax.grid(True, which='both', axis='y')
    ax.legend(ncol=3, loc='best', labels=new.mean().index.levels[0], title="Number of processes")
    if save:
        tikz_save(
        path+'dma_rept.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        extra_axis_parameters={'title style={align=center}'}
        #extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
        )
    else:
        plt.show()
    return


def show_stride(df):
    plt.cla()
    new = df.query('type == 8')
    new = new[['num_proc', 'time']].groupby(['num_proc'])
    #!!!
    plt.bar(range(1,len(new.median().index.values)+1), new.median()['time']/2, tick_label= new.median().index.values.tolist(), align='center')
    ax = plt.gca();
    ax.set_xlabel("Number of processes")
    ax.set_ylabel("Median time for one DMA transaction [ns]")
    ax.set_title(r"DMA vector copy with SVM\\Strided 4K access")
    ax.grid(True, which='both', axis='y')
    '''
    tikz_save(
    'mytikz.tex',
    figureheight='\\figureheight',
    figurewidth='\\figurewidth',
    extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
    )
    '''
    plt.show()
    return

def show_stride_odp(df):
    plt.cla()
    new = df.query('type == 9')
    new = new[['num_proc', 'time']].groupby(['num_proc'])
    #!!!
    plt.bar(range(1,len(new.median().index.values)+1), new.median()['time']/2, tick_label= new.median().index.values.tolist(), align='center', log=False)
    ax = plt.gca();
    ax.set_xlabel("Number of processes")
    ax.set_ylabel("Median time for one DMA transaction [ns]")
    ax.set_title(r"DMA vector copy with SVM+ODP\\Strided 4K access")
    ax.grid(True, which='both', axis='y')
    '''
    tikz_save(
    'mytikz.tex',
    figureheight='\\figureheight',
    figurewidth='\\figurewidth',
    extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
    )
    '''
    plt.show()
    return

def show_stride_rept(df):
    plt.cla()
    new = df.query('type == 10')
    print(new)
    new = new[['num_proc', 'num_runs', 'time']].groupby(['num_proc', 'num_runs'])
    print(new.mean())
    new_mean = new.mean()
    #!!!
    new_mean = new_mean/2
    ax = new_mean.unstack(0).plot(logy=True)
    ax.set_xlabel("Number of times same VMA was reused")
    ax.set_ylabel("Mean time for DMA transaction [ns]")
    ax.set_title(r"DMA vector copy with SVM\\Strided 4K access\\Reuse of same VMA")
    ax.grid(True, which='both', axis='y')
    ax.legend(ncol=3, loc='best', labels=new.mean().index.levels[0], title="Number of processes")
    '''
    tikz_save(
    'mytikz.tex',
    figureheight='\\figureheight',
    figurewidth='\\figurewidth',
    extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
    )
    '''
    plt.show()
    return

def show_bare(df, save):
    plt.cla()
    new = df.query('type == 11')
    new = new[['num_byte', 'time']].groupby(['num_byte'])
    new_med = new.median()
    #!!!
    new_med.index = new_med.index*2
    ax = new_med.plot(logy=True, logx=True)
    ax.set_xlabel("Bytes transferred")
    ax.set_ylabel("Bandwidth [MB/s]")
    ax.set_title(r"DMA vector copy Bare-Metal")
    ax.set_ylim(bottom=98, top=4000)
    ax.grid(True, which='both', axis='y')
    ax.get_legend().remove()
    if save:
        tikz_save(
        path+'dma_bare.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        #extra_axis_parameters={'title style={align=center}'}
        extra_axis_parameters={'extra y ticks=100',  'title style={align=center}'}
        )
    else:
        plt.show()

    return

def show_comparison_4burst(df):
    plt.cla()
    norm = drop(df.query('type == 0')).mean().xs(1, level=0)
    odp = drop(df.query('type == 1')).mean().xs(1, level=0)
    phys = drop(df.query('type == 2')).mean().xs(1, level=0)
    bare = df.query('type == 11')[['num_byte', 'time']].groupby(['num_byte']).max()
    
    #!!!
    plt.plot(norm.index*2, 32000/norm, label='Normal')
    plt.plot(odp.index*2, 32000/odp, label='ODP')
    plt.plot(phys.index*2, 32000/phys, label='Phys')
    plt.plot(bare.index*2, 32000/bare.values, label='Bare')
    ax = plt.gca()    
    ax.set_xscale("log")
    ax.set_xlabel("Bytes transferred")
    ax.set_ylabel("Bandwidth [MB/s]")
    ax.set_title(r"DMA vector copy comparison for 1 process")
    ax.legend()
    ax.grid(True, which='both', axis='y')
    '''
    tikz_save(
    'mytikz.tex',
    figureheight='\\figureheight',
    figurewidth='\\figurewidth',
    extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
    )
    '''
    plt.show()
    return

def show_comparison_percentage_4burst(df, save):
    plt.cla()
    norm = drop(df.query('type == 0')).mean().xs(1, level=0)
    odp = drop(df.query('type == 1')).mean().xs(1, level=0)
    phys = drop(df.query('type == 2')).mean().xs(1, level=0)
    bare = df.query('type == 11')[['num_byte', 'time']].groupby(['num_byte']).max()
    
    #!!!
    bare = 32000/bare
    plt.plot(norm.index*2, 100*(32000/norm)/bare, label='SVM')
    plt.plot(odp.index*2,  100*(32000/odp)/bare, label='SVM+ODP')
    plt.plot(phys.index*2, 100*(32000/phys)/bare, label='w/o SVM')
    plt.plot(bare.index*2, 100*bare/bare, label='Bare')
    ax = plt.gca()    
    ax.set_xscale("log")
    ax.set_xlim(left=32000)
    ax.set_ylim(top=150)
    ax.set_xlabel("Bytes transferred")
    ax.set_ylabel("% of Baremetal Bandwidth")
    ax.set_title(r"DMA vector copy performance comparison for 1 process")
    ax.legend()
    ax.grid(True, which='both', axis='y')
    if save:
        tikz_save(
        path+'dma_comp_4burst.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        extra_axis_parameters={'title style={align=center}'}
        #extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
        )
    else:
        plt.show()
    return

def show_comparison(df):
    plt.cla()
    norm = drop(df.query('type == 0')).mean().xs(1, level=0)
    odp = drop(df.query('type == 1')).mean().xs(1, level=0)
    phys = drop(df.query('type == 2')).mean().xs(1, level=0)
    bare = df.query('type == 11')[['num_byte', 'time']].groupby(['num_byte']).max()
    
    #!!!
    plt.plot(norm.index*2, 32000/norm, label='Normal')
    plt.plot(odp.index*2, 32000/odp, label='ODP')
    plt.plot(phys.index*2, 32000/phys, label='Phys')
    plt.plot(bare.index*2, 32000/bare.values, label='Bare')
    ax = plt.gca()    
    ax.set_xscale("log")
    ax.set_xlabel("Bytes transferred")
    ax.set_ylabel("Bandwidth [MB/s]")
    ax.set_title(r"DMA vector copy comparison for 1 process")
    ax.legend()
    ax.grid(True, which='both', axis='y')
    '''
    tikz_save(
    'mytikz.tex',
    figureheight='\\figureheight',
    figurewidth='\\figurewidth',
    extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
    )
    '''
    plt.show()
    return

def show_comparison_percentage(df, save):
    plt.cla()
    norm = drop(df.query('type == 0')).mean().xs(1, level=0)
    odp = drop(df.query('type == 1')).mean().xs(1, level=0)
    phys = drop(df.query('type == 2')).mean().xs(1, level=0)
    bare = df.query('type == 11')[['num_byte', 'time']].groupby(['num_byte']).max()
    
    #!!!
    plt.plot(norm.index*2, 100*norm/bare, label='SVM')
    plt.plot(odp.index*2,  100*odp/bare, label='SVM+ODP')
    plt.plot(phys.index*2, 100*phys/bare, label='w/o SVM')
    plt.plot(bare.index*2, 100*bare/bare, label='Bare')
    ax = plt.gca()    
    ax.set_xscale("log")
    ax.set_xlim(left=32000)
    ax.set_ylim(top=150)
    ax.set_xlabel("Bytes transferred")
    ax.set_ylabel("% of Baremetal Bandwidth")
    ax.set_title(r"DMA vector copy performance comparison for 1 process")
    ax.legend()
    ax.grid(True, which='both', axis='y')
    if save:
        tikz_save(
        path+'dma_comp.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        extra_axis_parameters={'title style={align=center}'}
        #extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
        )
    else:
        plt.show()
    return

def plot_async(save):
    plt.cla()
    data = "results_async.txt"
    df = pd.read_csv(data, header=0, sep=", ", engine="python")
    data = df.groupby(['total_byte']).median()
    
    plt.plot(data.index, data)
    ax = plt.gca()
    ax.set_xscale("log")
    ax.set_xlabel("Total bytes transferred")
    ax.set_ylabel("Bandwidth [MB/s]")
    ax.set_title(r"Async DMA + CPU vector copy performance for 1 process")
    ax.grid(True, which='both', axis='y')
    if save:
        tikz_save(
        path+'dma_async.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        extra_axis_parameters={'title style={align=center}'}
        #extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
        )
    else:
        plt.show()
    return

def plot_tlb(save):
    plt.cla()
    data = "results_inv_range.txt"
    range = pd.read_csv(data, header=0, sep=", ", engine="python")
    range = range.groupby(['svm', 'num_pages']).median()

    data = "results_inv_all.txt"
    all = pd.read_csv(data, header=0, sep=", ", engine="python").groupby(['num_pages']).median()
   
    #print(range.xs(0, level=0).index)
    #print(range.xs(0, level=0)/2)

    plt.plot(range.xs(0, level=0).index, range.xs(0, level=0)/2, label="Regular free")
    plt.plot(range.xs(1, level=0).index, range.xs(1, level=0)/2, label=r"free + mmu\_notifier + tlb\_inv\_range")
    plt.plot(range.xs(2, level=0).index, range.xs(2, level=0), label=r"tlb\_inv\_range")
    plt.plot(all.index, all, label="tlb\_inv\_context")
  
    print((100*(range.xs(1, level=0)/2)/(range.xs(0, level=0)/2)).median(axis=0))
    print((100*(range.xs(2, level=0))/(range.xs(0, level=0)/2)).median(axis=0))
    
    ax = plt.gca()
    ax.legend()
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Number of pages to be freed")
    ax.set_ylabel("Time [ns]")
    ax.set_title(r"Time to free memory")
    ax.grid(True, which='both', axis='y')
    if save:
        tikz_save(
        path+'tlb.tex',
        figureheight='\\figureheight',
        figurewidth='\\figurewidth',
        extra_axis_parameters={'title style={align=center}'}
        #extra_axis_parameters={'ytick distance=10', 'extra y ticks=50', 'title style={align=center}'}
        )
    else:
        plt.show()
    return


#types: 0=normal, 1=odp, 2=phys, 3=split, 4=repeat, 5=once_odp,
# 6=once_norm, 7=once_rep, 8=stride, 9=stride_odp, 10=stride_rep, 11=baremetal
def main():
    data = "results_100_16.txt"
    df = pd.read_csv(data, header=0, sep=", ", engine="python")
    df = remove_proc(df)
  
    save = False
    #show_normal(df, save)
    #show_odp(df, save)
    #show_phys(df, save)
    show_rept(df, True)
    #show_bare(df, save)
    #df = remove_all_proc(df)
    #show_comparison_percentage(df, save)
    #show_comparison(df)
    '''
    data = "results_100_4.txt"
    df = pd.read_csv(data, header=0, sep=", ", engine="python")
    df = remove_all_proc(df)
    show_comparison_percentage_4burst(df, save)
    show_comparison_4burst(df)
    '''
    #plot_async(save)    
    #plot_tlb(save)

    return

    #Liste Vergleich mean, median, num_proc=12
    print(pd.concat([odp_none_mean, odp_none_median, odp_none_stdv], axis=1).xs(12, level='num_proc').to_latex())
    
    print(odp_none_median.index.levels[0])



if __name__ == "__main__":
    main()
