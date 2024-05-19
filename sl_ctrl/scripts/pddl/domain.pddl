;Header and description

(define (domain shoe)

;remove requirements that are not needed
; (:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)
(:requirements :strips :negative-preconditions :typing :equality :conditional-effects)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    eyelet site - location
    eyelet_left eyelet_right - eyelet
    site_left site_right - site
    aglet
)

; un-comment following line if constants are needed
(:constants 
    aglet_a aglet_b - aglet
    ; site_left site_right - site
    site_l1 site_l2 site_l3 - site_left
    site_r1 site_r2 site_r3 - site_right
)

(:predicates ;todo: define predicates here
    ; (cursor_a ?el - eyelet)
    ; (cursor_b ?el - eyelet)    
    (aglet_at ?al - aglet ?s - site)
    (eyelet_laced ?el - eyelet)
    (block ?l1 - location ?l2 - location)
    (lace_with ?e - eyelet ?a - aglet)
    (covered ?s - site)
    (occupied ?s - site)
    (at_correct_side ?a - aglet)
    (left_correct ?a - aglet)
    (right_correct ?a - aglet)
)

; (:functions ;todo: define numeric functions here
;     (id ?el - eyelet)
; )

; ACTION DEFINITIONS
(:action left_insert_a
    :parameters (?el - eyelet_left ?s1 - site_left ?s2 - site_right)
    :precondition (and
        (not (eyelet_laced ?el)) ; not laced yet
        (lace_with ?el aglet_a) ; is supposed to lace with a
        (aglet_at aglet_a ?s1) ; aglet is at picking site
        (not (covered ?s1)); site is not covered
        (not (aglet_at aglet_b ?s2)) ; placing site is not taken
        (not (aglet_at aglet_a site_l3)) ; l3 empty (which blocks eyelets)
        (not (aglet_at aglet_b site_l3)) ; l3 empty (which blocks eyelets)
        (at_correct_side aglet_b)
        (forall (?e - eyelet)
            (not (block ?el ?e)) ; this eyelet does not block other eyelets
        )
    )
    :effect (and
        (not (aglet_at aglet_a ?s1))
        (aglet_at aglet_a ?s2)
        (eyelet_laced ?el) ; eyelet laced
        (left_correct aglet_a)
        (not (right_correct aglet_a))
        (not (at_correct_side aglet_a))
        (forall (?e - eyelet)
            (not (block ?e ?el)) ; other eyelets cannot block this eyelet
        )
        (when 
            (not (= ?s1 ?s2)) ; if not same site, change it
            (and (aglet_at aglet_a ?s1) (not (aglet_at aglet_a ?s2)))
        )
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_r3))
            (and (not (covered site_r1)) (not (covered site_r2)))
        )
    )
)

(:action left_insert_b
    :parameters (?el - eyelet_left ?s1 - site_left ?s2 - site_right)
    :precondition (and
        (not (eyelet_laced ?el)) ; not laced yet
        (lace_with ?el aglet_b) ; is supposed to lace with b
        (aglet_at aglet_b ?s1) ; aglet is at picking site
        (not (covered ?s1)); site is not covered
        (not (aglet_at aglet_a ?s2)) ; placing site is not taken
        (not (aglet_at aglet_a site_l3)) ; l3 empty (which blocks eyelets)
        (not (aglet_at aglet_b site_l3)) ; l3 empty (which blocks eyelets)
        (at_correct_side aglet_a)
        (forall (?e - eyelet)
            (not (block ?el ?e)) ; this eyelet does not block other eyelets
        )
    )
    :effect (and
        (not (aglet_at aglet_b ?s1))
        (aglet_at aglet_b ?s2)
        (eyelet_laced ?el) ; eyelet laced
        (left_correct aglet_b)
        (not (right_correct aglet_b))
        (not (at_correct_side aglet_b))
        (forall (?e - eyelet)
            (not (block ?e ?el)) ; other eyelets cannot block this eyelet
        )
        (when 
            (not (= ?s1 ?s2)) ; if not same site, change it
            (and (aglet_at aglet_b ?s1) (not (aglet_at aglet_b ?s2)))
        )
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_r3))
            (and (not (covered site_r1)) (not (covered site_r2)))
        )
    )
)

(:action right_insert_a
    :parameters (?el - eyelet_right ?s1 - site_right ?s2 - site_left)
    :precondition (and
        (not (eyelet_laced ?el)) ; not laced yet
        (lace_with ?el aglet_a) ; is supposed to lace with a
        (aglet_at aglet_a ?s1) ; aglet is at picking site
        (not (covered ?s1)); site is not covered
        (not (aglet_at aglet_b ?s2)) ; placing site is not taken
        (not (aglet_at aglet_a site_r3)) ; r3 empty (which blocks eyelets)
        (not (aglet_at aglet_b site_r3)) ; r3 empty (which blocks eyelets)
        (at_correct_side aglet_b)
        (forall (?e - eyelet)
            (not (block ?el ?e)) ; this eyelet does not block other eyelets
        )
    )
    :effect (and
        (not (aglet_at aglet_a ?s1))
        (aglet_at aglet_a ?s2)
        (eyelet_laced ?el) ; eyelet laced
        (right_correct aglet_a)
        (not (left_correct aglet_a))
        (not (at_correct_side aglet_a))
        (forall (?e - eyelet)
            (not (block ?e ?el)) ; other eyelets cannot block this eyelet
        )
        (when 
            (not (= ?s1 ?s2)) ; if not same site, change it
            (and (aglet_at aglet_a ?s1) (not (aglet_at aglet_a ?s2)))
        )
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_l3))
            (and (not (covered site_l1)) (not (covered site_l2)))
        )
    )
)

(:action right_insert_b
    :parameters (?el - eyelet_right ?s1 - site_right ?s2 - site_left)
    :precondition (and
        (not (eyelet_laced ?el)) ; not laced yet
        (lace_with ?el aglet_b) ; is supposed to lace with b
        (aglet_at aglet_b ?s1) ; aglet is at picking site
        (not (covered ?s1)); site is not covered
        (not (aglet_at aglet_a ?s2)) ; placing site is not taken
        (not (aglet_at aglet_a site_r3)) ; r3 empty (which blocks eyelets)
        (not (aglet_at aglet_b site_r3)) ; r3 empty (which blocks eyelets)
        (at_correct_side aglet_a)
        (forall (?e - eyelet)
            (not (block ?el ?e)) ; this eyelet does not block other eyelets
        )
    )
    :effect (and
        (not (aglet_at aglet_b ?s1))
        (aglet_at aglet_b ?s2)
        (eyelet_laced ?el) ; eyelet laced
        (right_correct aglet_b)
        (not (left_correct aglet_b))
        (not (at_correct_side aglet_b))
        (forall (?e - eyelet)
            (not (block ?e ?el)) ; other eyelets cannot block this eyelet
        )
        (when 
            (not (= ?s1 ?s2)) ; if not same site, change it
            (and (aglet_at aglet_b ?s1) (not (aglet_at aglet_b ?s2)))
        )
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_l3))
            (and (not (covered site_l1)) (not (covered site_l2)))
        )
    )
)

(:action left_to_right_transfer
    :parameters (?al - aglet ?s1 - site_left ?s2 - site_right)
    :precondition (and 
        (aglet_at ?al ?s1)
        (not (occupied ?s2)); site is not covered
        (not (covered ?s1)); site is not covered
    )
    :effect (and 
        (not (aglet_at ?al ?s1))
        (aglet_at ?al ?s2)
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_r1) (occupied site_r2))
            (covered site_r2)
        )
        (when 
            (and (= ?s2 site_r2) (occupied site_r1))
            (covered site_r1)
        )
        (when (right_correct ?al) (at_correct_side ?al))
        (when (left_correct ?al) (not (at_correct_side ?al)))
    )
)

(:action right_to_left_transfer
    :parameters (?al - aglet ?s1 - site_right ?s2 - site_left)
    :precondition (and
        (aglet_at ?al ?s1)
        (not (occupied ?s2)); site is not covered
        (not (covered ?s1)); site is not covered
    )
    :effect (and
        (not (aglet_at ?al ?s1))
        (aglet_at ?al ?s2)
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_l1) (occupied site_l2))
            (covered site_l2)
        )
        (when 
            (and (= ?s2 site_l2) (occupied site_l1))
            (covered site_l1)
        )
        (when (left_correct ?al) (at_correct_side ?al))
        (when (right_correct ?al) (not (at_correct_side ?al)))
    )
)

(:action left_replace
    :parameters (?al - aglet ?s1 - site_left ?s2 - site_left)
    :precondition (and
        (aglet_at ?al ?s1)
        (not (occupied ?s2)); site is not covered
        (not (covered ?s1)); site is not covered
    )
    :effect (and
        (not (aglet_at ?al ?s1))
        (aglet_at ?al ?s2)
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_l1) (occupied site_l2))
            (covered site_l2)
        )
        (when 
            (and (= ?s2 site_l2) (occupied site_l1))
            (covered site_l1)
        )
    )
)

(:action right_replace
    :parameters (?al - aglet ?s1 - site_right ?s2 - site_right)
    :precondition (and
        (aglet_at ?al ?s1)
        (not (occupied ?s2)); site is not covered
        (not (covered ?s1)); site is not covered
    )
    :effect (and
        (not (aglet_at ?al ?s1))
        (aglet_at ?al ?s2)
        (not (occupied ?s1))
        (occupied ?s2)
        (when 
            (and (= ?s2 site_r1) (occupied site_r2))
            (covered site_r2)
        )
        (when 
            (and (= ?s2 site_r2) (occupied site_r1))
            (covered site_r1)
        )
    )
)


)