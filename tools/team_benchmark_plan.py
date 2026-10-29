"""Fixed comparison matrix. These names are shared by runner, analysis and guide."""
BASE = ['safe-euclidean', 'max', 'directional']
TEAM8 = [(e,v) for e in ['LBA','Mesh'] for v in BASE] + [
    ('Mesh','max-buckets'), ('Mesh','directional-buckets')]
EXTENDED10 = TEAM8 + [('LBA','max-buckets'), ('LBA','directional-buckets')]
MATRICES = {'team8': TEAM8, 'extended10': EXTENDED10}
SIGNATURE = ['status','cost','path_cost_double','path','steps','checked_cells',
             'candidates','pushes','pops','closed_discards','stale_discards','dominated','peak_open']

def combinations(matrix, engines, variants):
    return list(MATRICES[matrix]) if matrix else [(e,v) for e in engines for v in variants]

def model_key(task, engine, variant):
    return task + '|' + engine + '|' + variant.replace('-buckets','')
