use proc_macro::TokenStream;
use proc_macro2::TokenTree as TokenTree2;
use proc_macro2::{Group, Ident, TokenStream as TokenStream2};
use quote::{ToTokens, format_ident, quote, quote_spanned};
use syn::punctuated::Punctuated;
use syn::{Expr, FnArg, ItemConst, ItemFn, LitStr, Pat, Token, Type, parse_macro_input};

fn hide(attr: TokenStream, item: TokenStream, export: bool) -> TokenStream {
   let parser = Punctuated::<LitStr, Token![,]>::parse_terminated;
   let args = parse_macro_input!(attr with parser);
   let feature_str = args[0].value();
   let module: syn::Path = syn::parse_str(&args[1].value()).unwrap();

   // Parse the private function this attribute is attached to
   let mut input_fn = parse_macro_input!(item as ItemFn);
   let fn_name = input_fn.sig.ident;
   let new_fn_name = format_ident!("phantom_{}", fn_name);
   input_fn.sig.ident = new_fn_name.clone();

   // Extract the exact argument identifiers
   let mut arg_names = Vec::new();
   for arg in &input_fn.sig.inputs {
      if let FnArg::Typed(pat_type) = arg {
         // We specifically look for standard Ident patterns (e.g., `raw_dots: &[Dot]`)
         if let Pat::Ident(pat_ident) = &*pat_type.pat {
            arg_names.push(&pat_ident.ident);
         }
      }
   }

   // Generate the output token stream
   let mut expanded = quote! {
      // 1. Output the original function completely untouched

      #[cfg(feature = #feature_str)]
      #input_fn

     // 2. Generate the macro that calls the function if the feature is enabled
      #[cfg(feature = #feature_str)]
      macro_rules! #fn_name {
         // Notice how `quote` elegantly handles the comma-separated repetitions
         ( #( $#arg_names:expr ),* ) => {
            #module :: #new_fn_name( #( $#arg_names ),* )
         };
      }

       // 3. Generate the macro that expands to a no-op if the feature is disabled
      #[cfg(not(feature = #feature_str))]
      macro_rules! #fn_name {
         ( #( $#arg_names:expr ),* ) => {
            // Expanding to unit `()` ensures it acts safely as a no-op statement or expression.
            ()
         };
      }
   };

   if export {
      expanded.extend(quote! {pub(crate) use #fn_name;});
   }

   TokenStream::from(expanded)
}

#[proc_macro_attribute]
pub fn phantom(attr: TokenStream, item: TokenStream) -> TokenStream {
   hide(attr, item, false)
}

#[proc_macro_attribute]
pub fn phantom_pub(attr: TokenStream, item: TokenStream) -> TokenStream {
   hide(attr, item, true)
}

fn replace_ident_recursive(tokens: TokenStream2, target: &Ident, replacement: &Expr, ty: &Type) -> TokenStream2 {
   tokens
      .into_iter()
      .map(|token| {
         match token {
            // 1. Found the identifier -> Replace it
            TokenTree2::Ident(ref id) if id == target => {
               quote_spanned!(id.span()=> (#replacement as #ty))
            }

            // 2. Found a Group ( (), [], {} ) -> Recurse into it
            TokenTree2::Group(group) => {
               let stream = replace_ident_recursive(group.stream(), target, replacement, ty);
               let mut new_group = Group::new(group.delimiter(), stream);
               new_group.set_span(group.span());
               TokenTree2::Group(new_group).into()
            }

            // 3. Other tokens (Punctuation, Literals) -> Keep as is
            _ => token.into_token_stream(),
         }
      })
      .collect()
}

#[proc_macro_attribute]
pub fn process(attr: TokenStream, item: TokenStream) -> TokenStream {
   let mut input_const = parse_macro_input!(item as ItemConst);
   let attr_tokens = TokenStream2::from(attr);

   // Perform deep recursive replacement
   let new_init_tokens = replace_ident_recursive(attr_tokens, &input_const.ident, &input_const.expr, &input_const.ty);

   // Update the constant's expression
   input_const.expr = syn::parse2(new_init_tokens).expect("Generated invalid expression");
   input_const.to_token_stream().into()
}
