/**
 *  robotexplo
 *  Author: Maelia
 *  Description: 
 */
model robotexplo
import "platform:/plugin/msi.gama.gui.application/generated/std.gaml"
global {
	var exploration_pourcentage_fin type: int init: 100 min: 10 max: 100 parameter:
	'Pourcentage à explorer : ' category: 'General';
	var environnement_type type: string init: 'Ouvert' parameter: 'Type d environnement : '
	among: [ 'Ouvert','Batiment' ] category: 'General';
	var position_init type: string init: '1' parameter: 'Position initiale : ' among: [ '1'
	, '2' , '3' , '4' , '5' ] category: 'General';
	var x_max type: int init: environnement_type = 'Batiment' ? /*275*/ 445 : ( (
	environnement_type = 'Ouvert' ) ? 295 /*138 */ : 100 );
	var y_max type: int init: environnement_type = 'Batiment' ? /*112*/ 244 : ( (
	environnement_type = 'Ouvert' ) ? 287 /*144*/ : 100 );
	var dist_candidats type: float parameter: 'Distance entre 2 candidats' init:
	2.0 min: 1.0 category: 'Robot';
	var perception_distance type: int parameter: 'Distance de perception : ' init:
	10 min: 1 category: 'Robot';
	var strategie_explo type: string init:'Promethee'  among: [ 'Promethee' ,'Basilico',
	'Gonzalez-Banos' , 'distance_min' , 'Electre' , 'aleatoire' ] parameter:
	'Strategie d exploration : ' category: 'Robot';
	var lambda type: float parameter: 'Lambda (Gonzalez-Banos) : ' init: 0.2 min:
	0.0 max: 1.0 category: 'Gonzalez-Banos';
	var wCp_Bas type: float parameter: 'Poids critere Distance (Basilico) : '
	init: 0.4 min: 0.0 max: 1.0 category: 'Basilico';
	var wiarea_Bas type: float parameter:
	'Poids critere information area (Basilico) : ' init: 0.3 min: 0.0 max: 1.0
	category: 'Basilico';
	var wiseg_Bas type: float parameter:
	'Poids critere information seg (Basilico) : ' init: 0.3 min: 0.0 max: 1.0
	category: 'Basilico';
	var wOp_Bas type: float parameter: 'Poids critere Overlaps (Basilico) : '
	init: 0.0 min: 0.0 max: 1.0 category: 'Basilico';
	var map_bassilico type: map init: [ '1' :: 0.5 , '2' :: 0.3 , '3' :: 0.2
	, '1_2' :: 0.95 , '2_1' :: 0.95 , '1_3' :: 0.7 , '3_1' :: 0.7 , '2_3' :: 0.4 ,
	'3_2' :: 0.4 ];
	var wCp_Ele type: float parameter: 'Poids critere Distance (Electre) : ' init:
	0.7 min: 0.0 max: 1.0 category: 'Electre';
	var pCp_Ele type: float parameter: 'Preference critere Distance (Electre) : '
	init: 200 category: 'Electre';
	var qCp_Ele type: float parameter:
	'Indifference critere Distance (Electre) : ' init: 0 category: 'Electre';
	var vCp_Ele type: float parameter: 'Veto critere Distance (Electre) : ' init:
	300 category: 'Electre';
	var wiarea_Ele type: float parameter:
	'Poids critere information area (Electre) : ' init: 0.2 min: 0.0 max: 1.0
	category: 'Electre';
	var piarea_Ele type: float parameter:
	'Preference critere information area (Electre) : ' init: 200 category:
	'Electre';
	var qiarea_Ele type: float parameter:
	'Indifference critere information area (Electre) : ' init: 0 category:
	'Electre';
	var viarea_Ele type: float parameter:
	'Veto critere nformation area (Electre) : ' init: 200 category: 'Electre';
	var wiseg_Ele type: float parameter:
	'Poids critere information seg (Electre) : ' init: 0.1 category: 'Electre';
	var piseg_Ele type: float parameter:
	'Preference critere information seg (Electre) : ' init: 8 category: 'Electre';
	var qiseg_Ele type: float parameter:
	'Indifference critere information seg (Electre) : ' init: 0 category:
	'Electre';
	var viseg_Ele type: float parameter:
	'Veto critere information seg (Electre) : ' init: 12 category: 'Electre';
	var wOp_Ele type: float parameter: 'Poids critere Overlaps (Electre) : ' init:
	0.0 min: 0.0 max: 1.0 category: 'Electre';
	var pOp_Ele type: float parameter: 'Preference critere Overlaps (Electre) : '
	init: 100 category: 'Electre';
	var qOp_Ele type: float parameter:
	'Indifference critere Overlaps (Electre) : ' init: 10 category: 'Electre';
	var vOp_Ele type: float parameter: 'Veto critere Overlaps (Electre) : ' init:
	150 category: 'Electre';
	var fuzzy_cut_electre type: float parameter: 'Fuzzy cut (Electre) : ' init:
	0.99 min: 0.1 max: 1.0 category: 'Electre';
	var wCp_Pro type: float parameter: 'Poids critere Distance (Promethee) : '
	init: 0.6 min: 0.0 max: 1.0 category: 'Promethee';
	var pCp_Pro type: float parameter:
	'Preference critere Distance (Promethee) : ' init: 100 category: 'Promethee';
	var qCp_Pro type: float parameter:
	'Indifference critere Distance (Promethee) : ' init: 1 category: 'Promethee';
	var wiarea_Pro type: float parameter:
	'Poids critere information area (Promethee) : ' init: 0.4 min: 0.0 max: 1.0
	category: 'Promethee';
	var piarea_Pro type: float parameter:
	'Preference critere information area (Promethee) : ' init: 60 category:
	'Promethee';
	var qiarea_Pro type: float parameter:
	'Indifference critere nformation area (Promethee) : ' init: 10 category:
	'Promethee';
	var wiseg_Pro type: float parameter:
	'Poids critere information seg (Promethee) : ' init: 0.2 category:
	'Promethee';
	var piseg_Pro type: float parameter:
	'Preference critere information seg (Promethee) : ' init: 7 category:
	'Promethee';
	var qiseg_Pro type: float parameter:
	'Indifference critere information seg (Promethee) : ' init: 5 category:
	'Promethee';
	var wOp_Pro type: float parameter: 'Poids critere Overlaps (Promethee) : '
	init: 0.0 min: 0.0 max: 1.0 category: 'Promethee';
	var pOp_Pro type: float parameter:
	'Preference critere Overlaps (Promethee) : ' init: 100 category: 'Promethee';
	var qOp_Pro type: float parameter:
	'Indifference critere Overlaps (Promethee) : ' init: 10 category: 'Promethee';
	var criteres_electre type: list init: [ ];
	var criteres_promethee type: list init: [ ];
	const init_data type: matrix init: environnement_type = 'Batiment' ? file (
	'../images/batiment-3.png' /*'../images/batiment-2.png'*/ ) : file (
	'../images/open-3.png' /*'../images/open-3-296-288.png'*/ );
	var nb_a_explorer type: int;
	var cellules type: list of: cellule;
	var cellules_vides type: list of: cellule;
	var color_exploree type: rgb init: [ 200 , 255 , 200 ] const: true;
	var color_frontiere type: rgb init: [ 50 , 255 , 50 ] const: true;
	var color_candidat type: rgb init: /*'pink'*/ 'white' const: true;
	var le_robot type: robot;
	var mca_agent type: multicriteria_analyzer;
	init {
		set seed value: 10 ;
		ask target: cellule as list {
			set est_obstacle value: int ( ( init_data at { grid_x , grid_y } ) ) != - 1
			;
			set color value: est_obstacle ? 'black' : 'white' ;
		}
		/*let mapCp type: map value: [];
		put in: mapCp item: 'Cp' at: 'name';
		put in: mapCp item: wCp_Ele at: 'weight';
		put in: mapCp item: pCp at: 'p';
		put in: mapCp item: qCp at: 'q';
		put in: mapCp item: vCp at: 'v';
		add item: mapCp to: criteres_electre;
		
		let mapiarea type: map value: [];
		put in: mapiarea item: 'iarea' at: 'name';
		put in: mapiarea item: wiarea_Ele at: 'weight';
		put in: mapiarea item: piarea at: 'p';
		put in: mapiarea item: qiarea at: 'q';
		put in: mapiarea item: viarea at: 'v';
		add item: mapiarea to: criteres_electre;
		
		let mapiseg type: map value: [];
		put in: mapiseg item: 'iseg' at: 'name';
		put in: mapiseg item: wiseg_Ele at: 'weight';
		put in: mapiseg item: piseg at: 'p';
		put in: mapiseg item: qiseg at: 'q';
		put in: mapiseg item: viseg at: 'v';
		add item: mapiseg to: criteres_electre;
		
		let mapOp type: map value: [];
		put in: mapOp item: 'Op' at: 'name';
		put in: mapOp item: wOp_Ele at: 'weight';
		put in: mapOp item: pOp at: 'p';
		put in: mapOp item: qOp at: 'q';
		put in: mapOp item: vOp at: 'v';
		add item: mapOp to: criteres_electre;
		*/
		set criteres_electre value: [ [ 'name' :: 'Cp' , 'weight' :: wCp_Ele , 'p' ::
		pCp_Ele , 'q' :: qCp_Ele , 'v' :: vCp_Ele ] , [ 'name' :: 'iarea' , 'weight'
		:: wiarea_Ele , 'p' :: piarea_Ele , 'q' :: qiarea_Ele , 'v' :: viarea_Ele ]
		/*, ['name'::'iseg', 'weight'::wiseg_Ele, 'p'::piseg_Ele, 'q'::qiseg_Ele, 'v'::viseg_Ele], ['name'::'Op', 'weight'::wOp_Ele, 'p'::pOp_Ele, 'q'::qOp_Ele, 'v'::vOp_Ele]*/
		] ;
		//set criteres_promethee value: [['name'::'Cp', 'weight'::wCp_Pro, 'p'::pCp_Pro, 'q'::qCp_Pro], ['name'::'iarea', 'weight'::wiarea_Pro, 'p'::piarea_Pro, 'q'::qiarea_Pro]/*, ['name'::'iseg', 'weight'::wiseg_Pro, 'p'::piseg_Pro, 'q'::qiseg_Pro], ['name'::'Op', 'weight'::wOp_Pro, 'p'::pOp_Pro, 'q'::qOp_Pro]*/];
		set criteres_promethee value: [ [ 'name' :: 'Cp' , 'weight' :: wCp_Pro , 's'
		:: 10 , 'type' :: 'type_6' , 'p' :: pCp_Pro , 'q' :: qCp_Pro ] , [ 'name' ::
		'iarea' , 'weight' :: wiarea_Pro , 'p' :: piarea_Pro , 'q' :: qiarea_Pro ] /*,
		[ 'name' :: 'iarea_lt' , 'weight' :: wiseg_Pro , 'p' :: piseg_Pro , 'q' ::
		qiseg_Pro ]*/
		/*,['name'::'centralite', 'weight'::0.2, 'p'::500, 'q'::100], ['name'::'iseg', 'weight'::wiseg_Pro, 'p'::piseg_Pro, 'q'::qiseg_Pro], ['name'::'Op', 'weight'::wOp_Pro, 'p'::pOp_Pro, 'q'::qOp_Pro]*/
		] ; create species: multicriteria_analyzer;
		set mca_agent value: first ( multicriteria_analyzer as list ) ; set cellules
		value: list ( cellule ) ; set cellules_vides value: cellules where ! ( each .
		est_obstacle ) ; set nb_a_explorer value: length ( cellules_vides ) ; create
		species: robot {
			if condition: environnement_type = 'Ouvert' {
				set my_cell value: cellules_vides first_with ( ( each . grid_x = 193
				/*113*/ ) and ( each . grid_y = 58 /*0*/ ) ) ;
				else {
					if condition: environnement_type = 'Batiment' {
						if condition: position_init = '1' {
							set my_cell value: cellules_vides first_with ( ( each . grid_x = /*12*/
							94 ) and ( each . grid_y = /*34*/ 100 ) ) ;
							else {
								if condition: position_init = '2' {
									set my_cell value: cellules_vides first_with ( ( each . grid_x = 145 )
									and ( each . grid_y = 36 ) ) ;
									else {
										if condition: position_init = '3' {
											set my_cell value: cellules_vides first_with ( ( each . grid_x = 180
											) and ( each . grid_y = 80 ) ) ;
											else {
												if condition: position_init = '4' {
													set my_cell value: cellules_vides first_with ( ( each . grid_x =
													247 ) and ( each . grid_y = 17 ) ) ;
													else {
														if condition: position_init = '5' {
															set my_cell value: cellules_vides first_with ( ( each . grid_x =
															10 ) and ( each . grid_y = 80 ) ) ;
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
			add to: zone_exploree item: my_cell;
			set my_cell . color value: [ 200 , 255 , 200 ] ; set location value: my_cell
			. location ;
		}
		set le_robot value: first ( robot as list ) ;
	}
	reflex when: length ( le_robot . zone_exploree ) >= int ( (
	exploration_pourcentage_fin * nb_a_explorer ) / 100.0 ) {
		do action: tell {
			arg message value: 'Fin de l exploration : methode : ' + strategie_explo +
			' temps : ' + time;
		}
		do action: halt;
	}
}
environment width: x_max height: y_max {
	grid cellule width: x_max height: y_max neighbours: 4 torus: false {
		var est_obstacle type: bool;
		var dist type: int init: - 99;
		var color type: rgb;
		var a_ete_vue type: bool init: false;
	}
}
entities {
	species robot skills: [ ] {
		var my_cell type: cellule;
		var chemin type: list of: cellule init: [ ];
		var cible type: cellule init: nil;
		var zone_exploree type: list of: cellule init: [ ];
		var frontiere type: list of: cellule init: [ ];
		var candidats type: list of: cellule init: [ ];
		//var non_frontiere type: list of: cellule init: [];
		var percept_dist type: int value: perception_distance;
		aspect default {
			draw shape: circle size: 1 color: 'red';
		}
		reflex perception {
			let percept_complet type: list of: cellule value: self . perception_reelle [
			];
			//let percept_complet type: list of: cellule value: my_cell neighbours_at percept_dist;
			add item: my_cell to: percept_complet;
			ask target: percept_complet {
				set a_ete_vue value: true ;
			}
			let percept type: list of: cellule value: percept_complet where ! ( each .
			est_obstacle );
			let front type: list of: cellule value: [ ];
			ask target: percept {
				set color value: color_exploree ;
			}
			loop cel over: percept {
				if condition: ! ( empty ( ( cel neighbours_at 1 ) where ( each . color =
				rgb ( 'white' ) ) ) ) {
					add item: cel to: front;
					set cel . color value: color_frontiere ;
				}
			}
			set zone_exploree value: zone_exploree union percept ; 
			set frontiere value: frontiere union front ; let front2 type: list of: cellule value: ( frontiere
			) copy_between { 0 , length ( frontiere ) };
			loop cel over: front2 /*where ( each distance_to my_cell <= (
			perception_distance )) */{
				if condition: ( empty ( ( cel neighbours_at 1 ) where ( each . color = rgb
				( 'white' ) ) ) ) {
					if condition: ( cel in frontiere ) {
					//remove item: cel from: frontiere;
						remove from: frontiere index: ( frontiere index_of cel );
					}
					set cel . color value: color_exploree ;
				}
			}
		}
		action perception_reelle {
			let perception type: list of: cellule value: [ ];
			add to: perception item: my_cell;
			let cpt type: int value: 0;
			let neighb type: list of: cellule value: ( my_cell neighbours_at 1 );
			let est_fini type: bool value: false;
			let obstacles type: list of: cellule value: ( my_cell neighbours_at
			percept_dist ) where ( each . est_obstacle );
			loop while: ! ( est_fini ) {
				set cpt value: cpt + 1 ;
				let neighb2 type: list of: cellule value: [ ];
				let cel type: cellule value: nil;
				loop cel over: neighb {
					if condition: ! ( cel . est_obstacle ) {
						let vois type: list of: cellule value: ( cel neighbours_at 1 ) where ( !
						( each in neighb ) and ! ( each in neighb2 ) );
						loop cel_vois over: vois {
							let est_visible type: bool value: true;
							let ligne type: geometry value: line ( [ cel_vois . location , my_cell .
							location ] );
							loop obst over: obstacles {
								if condition: ( est_visible ) and ( ligne overlaps obst . shape ) {
									set est_visible value: false ;
								}
							}
							if condition: ( est_visible ) {
								add item: cel_vois to: neighb2;
							}
						}
					}
				}
				set neighb value: neighb2 ; set perception value: perception union neighb ;
				if condition: ( empty ( neighb ) ) or ( cpt = percept_dist ) {
					set est_fini value: true ;
				}
			}
			return value: perception;
		}
		/*reflex write {
			save to:(environnement_type + '_' + strategie_explo +'.txt') type: 'csv' item: [time, (float ( length ( le_robot .
			zone_exploree ) / nb_a_explorer ) * 100)];
		}*/
		reflex deplacement {
		/*do action: write {
				arg message value: 'cible : ' + cible + '  my_cell : ' + my_cell + '  chemin : ' + chemin;
			}*/
			if condition: ( cible = nil ) or ( my_cell = cible ) or ( empty ( chemin ) )
			{
				if condition: cible != nil {
					set cible . color value: color_exploree ;
					if condition: cible in frontiere {
					//remove item: cible from: frontiere;
						remove from: frontiere index: ( frontiere index_of cible );
					}
				}
				do action: define_cands;
				if condition: ( strategie_explo = 'aleatoire' ) {
					do action: select_cible_alea;
					else {
						if condition: ( strategie_explo = 'distance_min' ) {
							do action: select_cible_plus_proche;
							else {
								if condition: ( strategie_explo = 'Gonzalez-Banos' ) {
									do action: select_cible_Gonzalez_Banos;
									else {
										if condition: ( strategie_explo = 'Basilico' ) {
											do action: select_cible_Basilico2;
											else {
												if condition: ( strategie_explo = 'Electre' ) {
													do action: select_cible_Electre;
													else {
														if condition: ( strategie_explo = 'Promethee' ) {
															do action: select_cible_Promethee;
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
				set cible . color value: 'yellow' ; do action: select_chemin;
			}
			if condition: ! ( empty ( chemin ) ) {
				set my_cell value: last ( chemin ) ;
				let pt1 type: point value: (last ( chemin )).location;
				let pt2 type: point value: location;
				let geom type: geometry value:line([pt1, pt2]) buffer 1.0;
				/*if condition: !(empty (test as list)) {
					set geom value: geom union (first(test as list)).shape;
				ask target: test as list {
					do action: die;
				}
				}
				create species: test {
					set shape value: geom;
				} */
				
				set location value: my_cell . location ;
				//remove item: my_cell from: chemin;
				remove from: chemin index: ( length ( chemin ) - 1 );
				//set chemin value: chemin copy_between [0, length(chemin) - 2];
			}
		}
		action define_cands {
			ask target: candidats {
				set color value: color_exploree ;
			}
			loop times: length ( candidats ) {
				remove from: candidats index: 0;
			}
			/*if condition: length ( frontiere ) <= nb_candidats_max {
				set candidats value: frontiere copy_between { 0 , length ( frontiere ) } ;
				else {
					set candidats value: [ ] ;
					let dispo type: list of: cellule value: frontiere copy_between { 0 ,
					length ( frontiere ) };
						loop times: nb_candidats_max {
						let cand type: cellule value: one_of ( dispo );
						//remove item: cand from: dispo;
						remove index: (dispo index_of cand) from: dispo;
						add item: cand to: candidats;
					}
				}
			}*/
			loop cel over: frontiere {
			/*do action: write  {
					arg message value: 'cel : ' + cel;
				}*/
				if condition: ( empty ( candidats ) ) or ( candidats min_of ( each
				distance_to cel ) > dist_candidats ) {
					add item: cel to: candidats;
				}
			}
			ask target: candidats {
				set color value: color_candidat;
			}
			if condition: my_cell in candidats {
			//remove item: my_cell from: candidats;
				remove index: ( candidats index_of my_cell ) from: candidats;
			}
		}
		action select_cible_alea {
			set cible value: one_of ( candidats ) ;
		}
		action select_cible_plus_proche {
			let list_dist type: list of: int value: self . calcul_dist [ les_cels ::
			candidats ];
			let dist_min type: int value: list_dist at 0;
			set cible value: candidats at 0 ;
			do action: write {
				arg message value: 'time : ' + time + ' : ' + candidats;
			}
			loop i from: 1 to: length ( candidats ) - 1 {
				let dist type: int value: list_dist at i;
				if condition: dist < dist_min {
					set cible value: candidats at i ;
					set dist_min value: dist ;
				}
			}

			//set cible value: candidats with_min_of (each distance_to my_cell)  ; 
		}
		action calcul_dist {
			arg les_cels type: list;
			let les_dist type: list of: int value: [ ];
			loop times: length ( les_cels ) {
				add item: 0 to: les_dist;
			}
			let est_fini type: bool value: false;
			/*ask target: cellules_vides {
				set dist value: - 1 ;
			}*/
			ask target: cellules_vides {
				set dist value: - 2 ;
			}
			ask target: zone_exploree {
				set dist value: - 1 ;
			}
			let cpt type: int value: 0;
			let trouves type: list of: cellule value: [ ];
			loop times: length ( trouves ) {
				remove from: trouves index: 0;
			}
			set my_cell . dist value: 0 ; if condition: ( my_cell in les_cels ) {
				put item: 0 in: les_dist at: ( les_cels index_of my_cell );
				add item: my_cell to: trouves;
				if condition: length ( trouves ) = length ( les_cels ) {
					set est_fini value: true ;
				}
			}
			let neighb type: list of: cellule value: ( my_cell neighbours_at 1 ) where (
			each . dist = - 1 );
			let est_fini type: bool value: false;
			loop while: ! ( est_fini ) {
				set cpt value: cpt + 1 ;
				let neighb2 type: list of: cellule value: [ ];
				let cel type: cellule value: nil;
				loop cel over: neighb {
					if condition: ! ( est_fini ) {
						set cel . dist value: cpt ;
						if condition: ( ! ( cel in trouves ) ) and ( cel in les_cels ) {
							let la_dist type: int value: cpt;
							put item: la_dist in: les_dist at: ( les_cels index_of cel );
							add item: cel to: trouves;
							if condition: length ( trouves ) = length ( les_cels ) {
								set est_fini value: true ;
							}
						}
						if condition: ! ( est_fini ) {
							let clp type: cellule value: nil;
							set neighb2 value: neighb2 union ( ( cel neighbours_at 1 ) where ( each
							. dist = ( - 1 ) ) ) ;
						}
					}
				}
				set neighb value: neighb2 ; if condition: ( cpt > x_max + y_max ) {
					loop cel2 over: les_cels {
						if condition: ! ( cel2 in trouves ) {
							let la_dist type: int value: 999999999;
							put item: la_dist in: les_dist at: ( les_cels index_of cel2 );
							add item: cel2 to: trouves;
						}
					}
					set est_fini value: true ;
				}
			}
			return value: les_dist;
		}
		action select_cible_Gonzalez_Banos {
			let val_max type: float value: - 1;
			let list_dist type: list of: int value: self . calcul_dist [ les_cels ::
			candidats ];
			loop i from: 0 to: length ( candidats ) - 1 {
				let Ap type: int value: length ( ( ( candidats at i ) neighbours_at
				perception_distance ) where ! ( each . a_ete_vue ) );
				let Cp type: int value: list_dist at i;
				let val type: float value: Ap * exp ( ( - 1 ) * lambda * Cp );
				if condition: val > val_max {
					set val_max value: val ;
					set cible value: ( candidats at i ) ;
				}
			}
		}
		action select_cible_Basilico {
			let val_max type: float value: - 1;
			let vals_Cp type: list of: int value: self . calcul_dist [ les_cels ::
			candidats ];
			let vals_iarea type: list of: int value: [ ];
			let vals_iseg type: list of: int value: [ ];
			let vals_Op type: list of: int value: [ ];
			loop i from: 0 to: length ( candidats ) - 1 {
				let cel type: cellule value: ( candidats at i );
				let cels_visu type: list of: cellule value: ( cel neighbours_at
				perception_distance );
				let iarea type: int value: length ( cels_visu where ! ( each . a_ete_vue )
				);
				let iseg type: int value: length ( cels_visu where ( each in frontiere ) );
				let Op type: int value: length ( cels_visu where ( each . est_obstacle ) );
				/*do action: write {
					arg message value: 'my_cell : {'+ my_cell.grid_x + ',' + my_cell.grid_y + '} cel : {' + cel.grid_x + ',' + cel.grid_y + '} Cp : ' + (vals_Cp at i) + ' iarea : ' + iarea + ' iseg : ' + iseg + ' Op : ' + Op;
				}*/
				add item: iarea to: vals_iarea;
				add item: iseg to: vals_iseg;
				add item: Op to: vals_Op;
			}
			let minCp type: int value: min ( vals_Cp );
			let maxCp type: int value: max ( vals_Cp );
			let miniarea type: int value: min ( vals_iarea );
			let maxiarea type: int value: max ( vals_iarea );
			let miniseg type: int value: min ( vals_iseg );
			let maxiseg type: int value: max ( vals_iseg );
			let minOp type: int value: min ( vals_Op );
			let maxOp type: int value: max ( vals_Op );
			loop i from: 0 to: length ( candidats ) - 1 {
				let uCp type: float value: maxCp = minCp ? 0 : 1 - ( ( ( vals_Cp at i ) -
				minCp ) / ( maxCp - minCp ) );
				let uiarea type: float value: maxiarea = miniarea ? 0 : ( ( ( vals_iarea at
				i ) - miniarea ) / ( maxiarea - miniarea ) );
				let uiseg type: float value: maxiseg = miniseg ? 0 : ( ( ( vals_iseg at i )
				- miniseg ) / ( maxiseg - miniseg ) );
				let uOp type: float value: maxOp = minOp ? 0 : ( ( ( vals_Op at i ) - minOp
				) / ( maxOp - minOp ) );
				let val type: float value: ( wCp_Bas * uCp ) + ( wiarea_Bas * uiarea ) + (
				wiseg_Bas * uiseg ) + ( wOp_Bas * uOp );
				if condition: val > val_max {
					set val_max value: val ;
					set cible value: ( candidats at i ) ;
				}
			}
		}
		action select_cible_Basilico2 {
			let val_max type: float value: - 1;
			let vals_Cp type: list of: int value: self . calcul_dist [ les_cels ::
			candidats ];
			let vals_iarea type: list of: int value: [ ];
			let vals_iseg type: list of: int value: [ ];
			let vals_Op type: list of: int value: [ ];
			loop i from: 0 to: length ( candidats ) - 1 {
				let cel type: cellule value: ( candidats at i );
				let cels_visu type: list of: cellule value: ( cel neighbours_at
				perception_distance );
				let iarea type: int value: length ( cels_visu where ! ( each . a_ete_vue )
				);
				let iseg type: int value: 1; //length (cels_visu where (each in frontiere));
				//let Op type: int value: length (cels_visu where (each.est_obstacle));
				/*do action: write {
					arg message value: 'i : '+ i +'  Cp : ' + (vals_Cp at i) + ' iarea : ' + iarea ;
				}*/
				add item: iarea to: vals_iarea;
				add item: iseg to: vals_iseg;
				//add item: Op to: vals_Op;
			}
			let minCp type: int value: min ( vals_Cp );
			let maxCp type: int value: max ( vals_Cp );
			let miniarea type: int value: min ( vals_iarea );
			let maxiarea type: int value: max ( vals_iarea );
			let miniseg type: int value: min ( vals_iseg );
			let maxiseg type: int value: max ( vals_iseg );
			//let minOp type: int value: min (vals_Op);
			//let maxOp type: int value: max (vals_Op);
			let indMax type: int value: 0;
			loop i from: 0 to: length ( candidats ) - 1 {
				let uCp type: point value: { 1.0 , maxCp = minCp ? 0.0 : 1.0 - ( ( (
				vals_Cp at i ) - minCp ) / ( maxCp - minCp ) ) };
				let uiarea type: point value: { 2.0 , maxiarea = miniarea ? 0.0 : ( ( (
				vals_iarea at i ) - miniarea ) / ( maxiarea - miniarea ) ) };
				let uiseg type: point value: { 3.0 , maxiseg = miniseg ? 0.0 : ( ( (
				vals_iseg at i ) - miniseg ) / ( maxiseg - miniseg ) ) };
				//let uOp type: float value: maxOp = minOp ? 0 :(((vals_Op at i) - minOp) / (maxOp - minOp));
				let list_vals type: list of: float value: ( ( [ uCp , uiarea , uiseg ] )
				sort_by ( each . y ) );
			/*	do action: write {
					arg message value: 'list_vals : '+ list_vals;
				}*/
			/*do action: write {
					arg message value: 'i : '+ i +'  uCp : ' + uCp.y + ' uiarea : ' + uiarea.y ;
				}*/
				let val type: float value: 0;
				let nb type: int value: length ( list_vals ) - 1;
				loop i from: 0 to: nb {
					let vlj type: point value: list_vals at i;
					if condition: ( i = 0 ) {
						set val value: val + vlj . y ;
						else {
							let vlj_prec type: point value: list_vals at ( i - 1 );
							let index type: string value: '';
							if condition: ( i = nb ) {
								set index value: string ( int ( vlj . x ) ) ;
								else {
									let vlj_next type: point value: list_vals at ( i + 1 );
									set index value: string ( int ( vlj . x ) ) + '_' + string ( int (
									vlj_next . x ) ) ;
								}
							}
							set val value: val + ( ( vlj . y - vlj_prec . y ) * ( map_bassilico at
							index ) ) ;
							//	do action: write {
					//	arg message value: 'index : '+ index+ '  val : ' + map_bassilico at index;
					//}
						}
					}
				}
				/*do action: write {
					arg message value:'i : '+ i + '   uCp : '+ uCp.y + '  uiarea : ' + uiarea.y + '  uiseg : ' + uiseg.y + '  val : ' +val;
				}*/
				if condition: val > val_max {
					set val_max value: val ;
					set cible value: ( candidats at i ) ;
					set indMax value: i;
				}
			}
		/*	do action: write {
				arg message value: 'val_max : '+ val_max + '  cible : ' + indMax;
			}*/
		}
		action select_cible_Electre {
			let cand_vals type: list value: [ ];
			let vals_Cp type: list of: int value: self . calcul_dist [ les_cels ::
			candidats ];
			loop i from: 0 to: length ( candidats ) - 1 {
				let cel type: cellule value: ( candidats at i );
				let cels_visu type: list of: cellule value: ( cel neighbours_at
				perception_distance );
				let iarea type: float value: float ( length ( cels_visu where ! ( each .
				a_ete_vue ) ) );
				//let iseg type: float value:  float(length (cels_visu where (each in frontiere)));
				//let Op type: float value:  float(length ((cel neighbours_at perception_distance) where (each.est_obstacle)));
				//do action: write {
				//	arg message value: 'i : ' + i +  ' dist : ' + float(x_max + y_max - (vals_Cp at i)) + '  iarea : ' + iarea /*+ '  iseg: ' + iseg*/;
				//}
				add to: cand_vals item: [ float ( x_max + y_max - ( vals_Cp at i ) ) , iarea
				/*, iseg*/ ];
			} 
			let index type: int value: - 1;
			ask target: mca_agent {
				set index value: self . electre_DM [ candidates :: cand_vals , criteria ::
				criteres_electre , fuzzy_cut :: fuzzy_cut_electre ] ;
			}
			/*do action: write {
				arg message value: 'index  : ' + index; 
			}*/
			set cible value: candidats at index ;
		}
		action select_cible_Promethee {
			let cand_vals type: list value: [ ];
			let vals_Cp type: list of: int value: self . calcul_dist [ les_cels ::
			candidats ];
			/*let dist_max type: float value: float(length(candidats) - 1) * sqrt(((x_max * x_max) + (y_max * y_max)) );
			do action: write {
				arg message value: 'dist_max :' + dist_max;
			}*/
/*ask target: (test as list) {
				do action: die;
			}*/
			loop i from: 0 to: length ( candidats ) - 1 {
				let cel type: cellule value: ( candidats at i );
				let iarea type: float value: float ( length ( ( cel neighbours_at (
				perception_distance ) ) where ! ( each . a_ete_vue ) ) );
				let iseg type: int value: 1; //length (cels_visu where (each in frontiere));
				//let iseg type: int value: 1; //length (cels_visu where (each in frontiere));
				
				let iarea_lt type: float value: float (length (( cel neighbours_at (
				perception_distance )) where (each in frontiere))); /*length ( ( cel neighbours_at ( 2 *
				perception_distance ) ) where ! ( each . a_ete_vue ) ) );*/
				/*if condition: i = 0 {
					let geomList type: list of: geometry value: ((cel neighbours_at (3 * perception_distance)) where !(each.a_ete_vue)) collect (each.shape) ;
					loop geom over: geomList {
						create species: test {
							set shape value: geom; 
						}
					}
				} */

//let centralite type: float value: -1 * sum (candidats collect (cel distance_to each));
				//let centralite type: float value: float(sum ((candidats where ((cel distance_to each) < perception_distance)) collect length((each neighbours_at perception_distance) where !(each.a_ete_vue))));
				//let iseg type: float value:  float(length ((cel neighbours_at perception_distance) where (each in frontiere)));
				//let Op type: float value:  float(length ((cel neighbours_at perception_distance) where (each.est_obstacle)));
				do action: write {
					arg message value: 'i : ' + i + ' dist : ' + float ( x_max + y_max - (
					vals_Cp at i ) ) + '  iarea : ' + iarea + '  iarea_lt : ' + iarea_lt
					/*+ '  iseg: ' + iseg*/;
				}
				add to: cand_vals item: [ float ( x_max + y_max - ( vals_Cp at i ) ) ,
				iarea , iarea_lt /*, iseg, Op*/ ];
			}
			let index type: int value: - 1;
			ask target: mca_agent {
				set index value: self . promethee_DM [ candidates :: cand_vals , criteria
				:: criteres_promethee ] ;
			}
			do action: write {
				arg message value: 'index  : ' + index;
			}
			set cible value: candidats at index ;
		}
		action select_chemin {
			set chemin value: [ ] ;
			if condition: cible != my_cell {
			/*ask target: cellules_vides {
					set dist value: - 1 ;
				}*/
				ask target: cellules_vides {
					set dist value: - 2 ;
				}
				ask target: zone_exploree {
					set dist value: - 1 ;
				}
				let cpt type: int value: 0;
				set my_cell . dist value: 0 ;
				let neighb type: list of: cellule value: ( my_cell neighbours_at 1 ) where
				( each . dist = - 1 );
				let est_fini type: bool value: false;
				loop while: ! ( est_fini ) {
					set cpt value: cpt + 1 ;
					let neighb2 type: list of: cellule value: [ ];
					let cel type: cellule value: nil;
					loop cel over: neighb {
						if condition: ! ( est_fini ) {
							set cel . dist value: cpt ;
							if condition: cel = cible {
								add item: cel to: chemin;
								let cel2 type: cellule value: cel;
								loop while: ( cpt > 0 ) {
									set cpt value: cpt - 1 ;
									set cel2 value: one_of ( ( cel2 neighbours_at 1 ) where ( each . dist
									= cpt ) ) ;
									add to: chemin item: cel2;
								}
								set est_fini value: true ;
							}
							if condition: ! ( est_fini ) {
								let clp type: cellule value: nil;
								set neighb2 value: neighb2 union ( ( cel neighbours_at 1 ) where ( each
								. dist = ( - 1 ) ) ) ;
							}
						}
					}
					set neighb value: neighb2 ; if condition: ( cpt > x_max + y_max ) {
						set chemin value: [ ] ;
						set est_fini value: true ;
					}
				}
				if condition: my_cell in chemin {
				//remove item: my_cell from: chemin;
					remove index: ( chemin index_of my_cell ) from: chemin;
				}
			}
		}
	}
	/*species test {
		aspect default {
			draw shape: geometry color: 'red';
		}
	}*/
}
output {
/*file results type: text data: string(time) +  ','+ (float ( length ( le_robot .
			zone_exploree ) / nb_a_explorer ) * 100);*/
	inspect name: 'Agents' type: agent refresh_every: 5;
	monitor name: 'zone_explo :' value: string ( float ( length ( le_robot .
	zone_exploree ) / nb_a_explorer ) * 100 );
	display main refresh_every: 1 {
		grid cellule;
		species robot;
		//species test;
	}
	/*display charts refresh_every: 5 {
		chart name: 'Pourcentage d exploration' type: series background: rgb ( 'white' ) {
			data exploration color: rgb ( 'red' ) value: float ( length ( le_robot .
			zone_exploree ) / nb_a_explorer ) * 100 style: area;
		}
	}*/
}