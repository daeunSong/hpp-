import matplotlib.pyplot as plt
import pickle

def readFromFile (fileName):
  data = []
  try:
      with open(fileName,'rb') as f:
        while True:
          try:
            line = pickle.load(f)
          except EOFError:
            break
          data.append(line)  
  except:
      return None
  return data[0]

############# main ###################    

if __name__ == '__main__':
    
    # fileName = "stepsize/stairs_1_ps"
    fileName = "new/rubbles_4_ps_c"
    data_ps = readFromFile(fileName)

    title = "[NEW] Rubbles_4: SL1M "
    COLOR = True
    
    fig, ax = plt.subplots()    
    
    s = data_ps[0]
    s_i = data_ps[1]
    s_f = data_ps[2]
    s_if = data_ps[3]
    p = data_ps[4]
    p_i = data_ps[5]
    p_f = data_ps[6]
    p_if = data_ps[7]
    # s = data_ps[0]
    # s_i = data_ps[1]
    # s_i_ = data_ps[2]
    # s_f = data_ps[3]
    # s_if = data_ps[4]
    # s_if_ = data_ps[5]
    # p = data_ps[6]
    # p_i = data_ps[7]
    # p_i_ = data_ps[8]
    # p_f = data_ps[9]
    # p_if = data_ps[10]
    # p_if_ = data_ps[11]    
    
    fail_color = ['gold','orange','red','gray']
    phase_fail = [[],[],[],[]]
    step_fail = [[],[],[],[]]
    
    for i in range(len(p_f[0])):
        phase = p_f[0][i]
        case = p_f[1][i]-1
        phase_fail[case] += [phase]
        step_fail[case] += [s_f[i]]
    
    ax.scatter(s, p, s=100, color='b', alpha=0.5, label = 'success')
    if COLOR:
        for i in range(len(phase_fail)):
            if step_fail[i] != []:
                ax.scatter(step_fail[i], phase_fail[i], s = 100, color=fail_color[i], alpha=0.5, label = 'fail case '+str(i+1))
    else:
        p_f = phase_fail[0]+phase_fail[1]+phase_fail[2]+phase_fail[3]        
        s_f = step_fail[0]+step_fail[1]+step_fail[2]+step_fail[3]
        ax.scatter(s_f, p_f, s = 100, color='r', alpha=0.5, label = 'fail')
    
    # ax.set_ylim([600,1400])

    ax.legend()
    ax.grid()
    plt.title(title + 'w/ continuous')
    plt.xlabel('step size')
    plt.ylabel('phase number')
    # plt.ylabel('slack scale')
    plt.ion()
    plt.show()
    
    fig, ax = plt.subplots()
    
    
    # phase_fail = [[],[],[],[]]
    # step_fail = [[],[],[],[]]
    
    # for i in range(len(p_if_[0])):
        # phase = p_if_[0][i]
        # case = p_if_[1][i]-1
        # phase_fail[case] += [phase]
        # step_fail[case] += [s_if_[i]]
    
    # ax.scatter(s_i_, p_i_, s=100, color='b', alpha=0.5, label = 'success')
    # if COLOR:
        # for i in range(len(phase_fail)):
            # if step_fail[i] != []:
                # ax.scatter(step_fail[i], phase_fail[i], s = 100, color=fail_color[i], alpha=0.5, label = 'fail case '+str(i+1))
    # else:
        # p_if_ = phase_fail[0]+phase_fail[1]+phase_fail[2]+phase_fail[3]        
        # s_if_ = step_fail[0]+step_fail[1]+step_fail[2]+step_fail[3]
        # ax.scatter(s_if_, p_if_, s = 100, color='r', alpha=0.5, label = 'fail')
    
    # # ax.set_ylim([600,1400])

    # ax.legend()
    # ax.grid()
    # plt.title(title + 'w/ intersection merged')
    # plt.xlabel('step size')
    # plt.ylabel('phase number')
    # # plt.ylabel('slack scale')
    # plt.ion()
    # plt.show()
    
    # fig, ax = plt.subplots()
    
    
    phase_fail = [[],[],[],[]]
    step_fail = [[],[],[],[]]
    
    for i in range(len(p_if[0])):
        phase = p_if[0][i]
        case = p_if[1][i]-1
        phase_fail[case] += [phase]
        step_fail[case] += [s_if[i]]
    
    ax.scatter(s_i, p_i, s = 100, color='b', alpha=0.5, label = 'success')
    if COLOR:
        for i in range(len(phase_fail)):
            if step_fail != []:
                ax.scatter(step_fail[i], phase_fail[i], s = 100, color=fail_color[i], alpha=0.5, label = 'fail case '+str(i+1))
    else:
        p_if = phase_fail[0]+phase_fail[1]+phase_fail[2]+phase_fail[3]   
        s_if = step_fail[0]+step_fail[1]+step_fail[2]+step_fail[3]     
        ax.scatter(s_if, p_if, s = 100, color='r', alpha=0.5, label = 'fail')
    
    # ax.set_ylim([600,1400])
    
    ax.legend()
    ax.grid()
    plt.title(title + ', w/ intersection')
    plt.xlabel('step size')
    plt.ylabel('phase number')
    # plt.ylabel('slack scale')
    plt.ion()
    plt.show()
        
