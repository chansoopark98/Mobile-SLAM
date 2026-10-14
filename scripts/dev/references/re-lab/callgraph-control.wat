(module
  (type $void (func))
  (import "host" "notify" (func $notify (type $void)))
  (table 1 funcref)
  (elem (i32.const 0) $leaf)
  (func $leaf (export "leaf") (type $void)
    call $notify)
  (func $entry (export "entry") (type $void)
    call $leaf
    call $leaf
    i32.const 0
    call_indirect (type $void)))
