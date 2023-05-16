import json
import matplotlib.pyplot as plt

METRICS = {'connected': [4, 8], 
           'size': [30, 50, 100], 
           'clump_size': ['small', 'medium', 'large'], 
           'coverage': [0.05, 0.1, 0.2],
           'percent_change': [0.05, 0.1, 0.2]}

def compare_ratios(lpa_star_results, lpara_star_results):
    ratios = []
    for (i, lpa_expanded) in enumerate(lpa_star_results):
        lpara_expanded = lpara_star_results[i]
        if (lpa_expanded['connected'] != lpara_expanded['connected'] or 
            lpa_expanded['size'] != lpara_expanded['size'] or 
            lpa_expanded['clump_size'] != lpara_expanded['clump_size'] or 
            lpa_expanded['coverage'] != lpara_expanded['coverage'] or 
            lpa_expanded['percent_change'] != lpara_expanded['percent_change']):
            raise ValueError('lpa and lpara conditions don\'t match')
        ratio = lpara_expanded['avg_states_expanded'] / lpa_expanded['avg_states_expanded']
        ratios.append({'connected': lpa_expanded['connected'], 
                    'size': lpa_expanded['size'], 
                    'clump_size': lpa_expanded['clump_size'],
                    'coverage': lpa_expanded['coverage'],
                    'percent_change': lpa_expanded['percent_change'],
                    'ratio': ratio})
        
    avg_ratio = sum([i['ratio'] for i in ratios]) / len(ratios)
    return ratios, avg_ratio

def get_breakdown_by_metric(ratios, metric):
    buckets = {}
    for stat in ratios:
        value = stat[metric]
        buckets[value] = buckets.get(value, []) + [stat['ratio']]
    for key in buckets:
        buckets[key] = sum(buckets[key]) / len(buckets[key])
    return buckets

def main():
    with open('data/validation_stats_lpa_star.json', 'r') as f:
        lpa_star_results = json.load(f)

    with open('data/validation_stats_lpara_star.json', 'r') as f:
        lpara_star_results = json.load(f)

    with open('data/validation_stats_8connected.json', 'r') as f:
        lpara_star_results8 = json.load(f)

    lpara_star_results = lpara_star_results[:len(lpara_star_results) // 2] + lpara_star_results8
    ratios, avg_ratio = compare_ratios(lpa_star_results, lpara_star_results)
    print(avg_ratio)
    json.dump(ratios, open('data/validation_ratios.json', 'w'))
    for metric in METRICS:
        stats = get_breakdown_by_metric(ratios, metric)
        print(metric, stats)
        plt.plot(list(stats.keys()), list(stats.values()))
        plt.title(metric)
        plt.ylim(0, 1)
        plt.xlabel('condition')
        plt.ylabel('ratio of LPARA* to LPA* expanded states')
        plt.show()

if __name__ == '__main__':
    main()
    