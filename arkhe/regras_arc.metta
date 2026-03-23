;; regras_arc.metta
;; Γ₁₂₈: Regras MeTTa para o ARC-AGI

;; Transformações geométricas
(= (rotate-90 $grid) (apply-rotation $grid 90))
(= (reflect $grid)   (apply-reflection $grid))

;; Composição de duas transformações
(= (compose $f $g $grid) ($f ($g $grid)))

;; Regra de descoberta de padrões via Syzygy
(= (find-pattern $grid)
   (let (($memory (memory-query $grid)))
        (match-arc-pattern $grid $memory)))

;; Verificação de Invariante C+F=1
(= (verify-arkhe-identity $node)
   (let (($c (get-coherence $node))
         ($f (get-fluctuation $node)))
        (if (== (+ $c $f) 1.0)
            True
            (Error "Identity Violation"))))
