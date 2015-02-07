{
    'variables': {
        'skia_os': 'mac',
    },
    
    'target_defaults': {
        'include_dirs': [
	    '../3rd',
	    '../src',
            '/usr/local/include',
            "<!(node -e \"require('nan')\")",
        ],
        
        'defines': [
            '_USE_MATH_DEFINES',
        ],
        'cflags_cc': [
            '-std=c++11',
	    '-frtti',
        ],
        
        'link_settings': {
            "ldflags": [
		'-I/usr/local/include',
		'-L/usr/local/lib',
		'-lboost_system',
                '-lboost_thread',
		'-lboost_date_time',
		'-lboost_timer',
                '-lboost_filesystem',
		'-lboost_serialization',
		'-lgdal',
		'-lgeos',
            ],
            
            'conditions': [
		['skia_os == "mac"', {
                    'cflags': [
			'-stdlib=libc++',
			'-frtti',
                    ],
		    'cflags_cc': [ '-frtti' ],
                    
                    "libraries": [
			'/Library/Frameworks/GEOS.framework',
                        '/Library/Frameworks/GDAL.framework',
			'/usr/local/lib/libboost_system-mt.a',
                        '/usr/local/lib/libboost_thread-mt.a',
                        '/usr/local/lib/libboost_filesystem-mt.a',
                        '/usr/local/lib/libboost_date_time-mt.a',
                        '/usr/local/lib/libboost_timer-mt.a',
                        '/usr/local/lib/libboost_chrono-mt.a',
			'/usr/local/lib/libboost_serialization-mt.a',
                    ],
		}]],
        },
        
    },
    'targets': [
	{
            'target_name': 'MMSolver',
#'type': 'static_library',
            'sources': [
		'../src/mm.h',
		'../src/mm.cpp',
		'../src/mm_density_solver.h',
		'../src/mm_density_solver.cpp',
		'../src/mm_graph.h',
		'../src/mm_graph.cpp',
		'../src/mm_route.h',
		'../src/mm_route.cpp',
		'../src/mm_sparse_solver.h',
		'../src/mm_sparse_solver.cpp',
		'../src/mm_tree.h',
		'../src/mm_tree.cpp',
		'../src/rl.h',
		'../src/rl.cpp',
		'../src/rl_utility.h',
		'../src/rl_utility.cpp',
		'../src/scale_model.h',
		'../src/scale_model.cpp',
		'mm_node.h',
		'mm_node.cc',
		'addon.cc',
            ],
	}
    ],
}
