import json

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

def main():
    with open('data/validation_stats_lpa_star.json', 'r') as f:
        lpa_star_results = json.load(f)

    with open('data/validation_stats_lpara_star.json', 'r') as f:
        lpara_star_results = json.load(f)

    with open('data/validation_stats_8connected.json', 'r') as f:
        lpara_star_results8 = json.load(f)

    lpara_star_results = lpara_star_results[:len(lpara_star_results) // 2] + lpara_star_results8
    ratios, avg_ratio = compare_ratios(lpa_star_results, lpara_star_results)
    json.dump(ratios, open('data/validation_ratios.json', 'w'))

if __name__ == '__main__':
    main()
    