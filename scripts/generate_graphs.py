import json
import matplotlib.pyplot as plt

def parse_evaluate_file(filepath):
    """
    Parse the 'evaluate' file line by line.
    Return a list of tuples (L, R, average_query_time).
    """
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            record = json.loads(line)
            if record.get("command") == "evaluate":
                config = record["configuration"]
                timings = record["timings"]
                
                L_value = config["L"]
                R_value = config["R"]
                avg_q_time = timings["averageQueryTime"]
                
                data.append((L_value, R_value, avg_q_time))
    return data

def main():
    # Replace with the path to your "evaluate" JSON file
    evaluate_file = "stats/plain-vamana-eval.json"
    
    # Parse the file
    eval_data = parse_evaluate_file(evaluate_file)
    
    # 1) For R=20, plot average query time vs. increasing L
    r_fixed = 20
    r20_data = [(L, t) for (L, R, t) in eval_data if R == r_fixed]
    # Sort by L
    r20_data.sort(key=lambda x: x[0])
    
    # Extract L and avg query time
    Ls_r20 = [item[0] for item in r20_data]
    avg_q_times_r20 = [item[1] for item in r20_data]
    
    # Plot 1
    plt.figure()
    plt.plot(Ls_r20, avg_q_times_r20, marker='o', linestyle='-', label='Vamana')
    
    # Add brute force line
    brute_force_time = 0.003
    plt.axhline(y=brute_force_time, color='red', linestyle='--', label='Brute Force')
    
    plt.xlabel('L')
    plt.ylabel('Average Query Time')
    plt.title(f'Average Query Time as L Increases (R={r_fixed})')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    # 2) For L=110, plot average query time vs. increasing R
    l_fixed = 110
    l110_data = [(R, t) for (L, R, t) in eval_data if L == l_fixed]
    # Sort by R
    l110_data.sort(key=lambda x: x[0])
    
    # Extract R and avg query time
    Rs_l110 = [item[0] for item in l110_data]
    avg_q_times_l110 = [item[1] for item in l110_data]
    
    # Plot 2
    plt.figure()
    plt.plot(Rs_l110, avg_q_times_l110, marker='o', linestyle='-', label='Vamana')
    
    # Add brute force line
    plt.axhline(y=brute_force_time, color='red', linestyle='--', label='Brute Force (0.003)')
    
    plt.xlabel('R')
    plt.ylabel('Average Query Time')
    plt.title(f'Average Query Time as R Increases (L={l_fixed})')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
