#!/usr/bin/env perl
# Usage: count_relative_eq.pl <file.rs> [<file.rs> ...]
#
# Round 17: p1-joints' `moveit-planners-pilz` copy of p3-acm's round-19
# `crates/moveit-geometry/audit/count_relative_eq.pl`, applied to the crate
# from its first round rather than deferred, per that round's instruction.
# Same classification: strips `/* */` block comments (including multi-line)
# and `//` line-comment tails, then blanks the contents of every `"..."`
# string literal (handles `\"` escapes; this crate's real call sites use no
# raw (`r"..."`) or byte (`b"..."`) string literals), then finds every
# remaining `assert_relative_eq!(` / `relative_eq!(` call by bracket-matching
# parens from the macro name to the matching close, and classifies each by
# whether `epsilon =` and/or `max_relative =` appear inside that call's own
# argument text.
use strict;
use warnings;
my ($both, $eps_only, $mr_only, $neither) = (0,0,0,0);
for my $file (@ARGV) {
    open(my $fh, '<', $file) or die "$file: $!";
    local $/;
    my $text = <$fh>;
    close $fh;
    $text =~ s{/\*.*?\*/}{}gs;
    $text =~ s{//[^\n]*}{}g;
    $text =~ s{"(?:[^"\\]|\\.)*"}{""}g;
    while ($text =~ /\b(?:assert_relative_eq|relative_eq)!\s*\(/g) {
        my $open = rindex($text, '(', pos($text) - 1);
        my $depth = 0;
        my $j = $open;
        my $len = length($text);
        for (; $j < $len; $j++) {
            my $c = substr($text, $j, 1);
            $depth++ if $c eq '(';
            $depth-- if $c eq ')';
            last if $depth == 0;
        }
        my $call = substr($text, $open, $j - $open + 1);
        my $has_eps = $call =~ /\bepsilon\s*=/;
        my $has_mr  = $call =~ /\bmax_relative\s*=/;
        if ($has_eps && $has_mr) { $both++ }
        elsif ($has_eps) { $eps_only++ }
        elsif ($has_mr) { $mr_only++ }
        else { $neither++ }
        printf STDERR "%s: call at byte %d -> eps=%d mr=%d\n", $file, $open, $has_eps, $has_mr;
    }
}
print "both=$both epsilon_only=$eps_only max_relative_only=$mr_only neither=$neither\n";
